/*
Copyright 2019 Andrey Petukhov
*/

#include <algorithm>
#include <assert.h>
#include <iostream>
#include <deque>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Huffman.h"

const int MAX_BYTE = 256;
const int BYTE_SIZE = 8;
const int END_OF_SEQ = -1;
// количество бит(длина) сериализованного дерева помещается в int
const size_t MAX_TREE_BYTE_LEN = sizeof(size_t);
const size_t MAX_PROBE = 1 << 24; // 4 Мегабайта (22), чтобы проверить итератор

// стандартный вектор
using std::vector;
// вектор байт
using list_byte = std::vector<byte>;
// неупорядоченное множество байт, используется для хранения символов ориг. последовательности
using set = std::unordered_set<byte>;
// пара (байт, число) используется для сортировки счетчиков
using tuple = std::tuple<byte, int>;
// пара (множество символов, число) используется построения дерева
using tuple_set = std::tuple<set, int>;
// словарь байт -> число используется для хранения частоты
using dict = std::unordered_map<byte, int>;

namespace huffman {

// big-endian ordering
// последовательность бит
// vector<bool>?
class BitSequence {
 public:
    // изначально пуста
    BitSequence(): bits(), bits_size(0) {}
    // преобразовывает количество бит в последовательность
    explicit BitSequence(size_t size);

    // добавляет к последовательности 1 бит
    BitSequence* append_bit(bool b);
    // добавляет к последовательности бит 1 байт (BYTE_SIZE бит)
    BitSequence* append(const byte& b);
    // добавляет к последовательности бит массив байт
    BitSequence* append(const list_byte& b);
    // добавляет к последовательности бит другую последовательность
    BitSequence* append(const BitSequence& seq);
    // сколько бит содержит последовательность
    size_t size() const { return bits_size; }
    // является ли последовательность пустой
    size_t empty() const { return size() == 0; }
    // сколько бит не занято в векторе
    size_t residual() const { return complement(bits_size); }
    // сколько байт в векторе
    size_t size_nbytes() const { return bits.size(); }
    // возвращает значение бита в позиции pos
    bool get_bit(size_t pos) const;
    // возвращает значение байта в позиции pos
    byte get_byte(size_t pos) const;
    // возвращает nbytes байт, начиная с позиции offset
    list_byte get_bytes(size_t offset, size_t nbytes) const;

    // преобразовывает последовательность в количество бит
    size_t seq2size() const;

    // сколько бит не хватает до полного байта
    static const size_t complement(size_t size);

 private:
    // вектор байт
    list_byte bits;
    // сколько содержит полезных бит
    size_t bits_size;
};

// словарь число -> последовательность бит, для кодировки символов
// END_OF_SEQ = конец последовательности символов, остальные число == код символа (unsigned)
using dict_seq = std::unordered_map<int, BitSequence>;

// Узел дерева
struct Node {
    // изначально пустой
    Node() : childs(), value(0), terminal(false) {}
    // терминальный узел (лист)
    explicit Node(int value) : childs(), value(value), terminal(true) {}
    Node* childs[2];  // левый и правый потомок
    int value;  // значение символа алфавита
    bool terminal;  // узел - лист
};

// пара используется для обхода дерева
using tuple_node = std::tuple<bool, Node*>;

// Дерево, содержащее оптимальный бинарный код Хаффмана
class Tree {
 public:
    // изначально пустое
    Tree(): root(nullptr) {}
    // сразу с корнем
    explicit Tree(Node* root): root(root) {}
    // удаление узлов в обратном обходе
    ~Tree();

    // декодирует последовательность бит в байты
    list_byte decode(const BitSequence& seq);
    // возвращает словарь символ -> последовательность бит для кодирования
    dict_seq get_map();
    // сохраняет дерево в последовательность бит
    BitSequence serialize();
    // строит дерево из последовательности бит
    void deserialize(const BitSequence& seq);
 private:
    Node* root;  // корень дерева
};

class Stream {
 public:
    // чтение максимум nbytes из потока from в список байт.
    list_byte read(IInputStream* from, size_t nbytes);
    // чтение максимум nbytes из потока from в последовательность бит.
    BitSequence read_seq(IInputStream* from, size_t nbytes);
    // Запись байта в поток
    void write(IOutputStream* to, const byte& b);
    // Запись последовательности бит в поток
    void write_seq(IOutputStream* to, const BitSequence& seq);
};

// Кодировщик потока
class Encoder: Stream {
 public:
    void encode(IInputStream* original, IOutputStream* compressed);

 protected:
    dict set_freq(const list_byte& v);  // вычисляет частоту символов в потоке
    Tree* build_tree(const dict& freq);  // Строит дерево с оптимальным кодом
};

// Раскодировщик потока
class Decoder: Stream {
 public:
    void decode(IInputStream* compressed, IOutputStream* original);
};

};  // namespace huffman

using huffman::BitSequence;
using huffman::Node;
using huffman::Tree;
using huffman::Stream;
using huffman::Encoder;
using huffman::Decoder;

using huffman::dict_seq;
using huffman::tuple_node;

// Кодирование потока original в поток compressed
void Encode(
    IInputStream& original,  // NOLINT(runtime/references)
    IOutputStream& compressed) {  // NOLINT(runtime/references)
    Encoder enc;
    enc.encode(&original, &compressed);
}

// Декодирование потока compressed в поток original
void Decode(
    IInputStream& compressed,  // NOLINT(runtime/references)
    IOutputStream& original) {  // NOLINT(runtime/references)
    Decoder dec;
    dec.decode(&compressed, &original);
}

// <------------ End of public interface

//============== Stream

// чтение максимум nbytes из потока from.
list_byte Stream::read(IInputStream* from, size_t nbytes) {
    byte value;
    list_byte v;
    while (nbytes && from->Read(value)) {
        v.push_back(value);
        nbytes--;
    }
    return std::move(v);
}

// чтение максимум nbytes из потока from в последовательность бит.
BitSequence Stream::read_seq(IInputStream* from, size_t nbytes) {
    BitSequence seq;
    byte value;
    while (nbytes && from->Read(value)) {
        seq.append(value);
        nbytes--;
    }
    return std::move(seq);
}

// Запись байта в поток
void Stream::write(IOutputStream* to, const byte& b) {
    to->Write(b);
}

// Запись последовательности бит в поток
void Stream::write_seq(IOutputStream* to, const BitSequence& seq) {
    for (size_t pos = 0; pos < seq.size_nbytes(); pos++ ) {
        write(to, seq.get_byte(pos));
    }
}

//============== end of Stream

//============== Encoder

// public high-level method
void Encoder::encode(IInputStream* original, IOutputStream* compressed) {
    list_byte orig = read(original, MAX_PROBE);
    dict freq = set_freq(orig);
    Tree* tree = build_tree(freq);
    BitSequence seq_tree = tree->serialize();
    BitSequence seq_tree_size = BitSequence(seq_tree.size_nbytes());
    write_seq(compressed, seq_tree_size);
    write_seq(compressed, seq_tree);
    dict_seq mymap = tree->get_map();
    delete tree;
    BitSequence* seq = new BitSequence;
    for (auto it: orig) {
        seq->append(mymap[it]);
    }
    seq->append(mymap[END_OF_SEQ]);
    write_seq(compressed, *seq);
    delete seq;
}


// Определение частоты символов в потоке
dict Encoder::set_freq(const list_byte& v) {
    dict freq;
    for (int c = 0; c < MAX_BYTE; c++) {
        freq[c] = 0;
    }
    for (auto it: v) {
        freq[it]++;
    }
    return std::move(freq);
}

// Построение дерева
Tree* Encoder::build_tree(const dict& freq) {
    vector<tuple_set> counter; // вектор пар (символ, счетчик) для сортировки
    vector<tuple_set> counter_sum; // вектор пар (набор символов, счетчик) в порядке возрастания

    std::unordered_map<int, Node*> nodes_counter;  // узлы с исходными символами
    vector<Node*> nodes_counter_sum;  // узлы с суммами частот (предки исходных узлов)

    tuple_set empty = std::make_tuple(set(), 0);

    // Добавим несуществующий элемент с частотой 0
    // это будет признаком конца потока, чтобы не получить коллизий
    // когда будем выравнивать последовательность бит до байта (дописав нули)
    counter.push_back(empty);
    counter_sum.push_back(empty);
    nodes_counter[END_OF_SEQ] = new Node(END_OF_SEQ);
    nodes_counter_sum.push_back(new Node);

    for (auto it: freq) {
        // first == key, second == value
        // filter non-zeros
        if (it.second == 0) {
            // пропускаем символы, которых нет в потоке
            continue;
        }
        // множество s состоит из 1 исходного символа
        set s({it.first});
        tuple_set t1 = std::make_tuple(s, it.second);
        counter.push_back(t1);

        // counter_sum изначально пустой
        counter_sum.push_back(empty);

        // узел с исходным символом
        Node* node = new Node(it.first);
        nodes_counter[it.first] = node;

        // пустой узел
        nodes_counter_sum.push_back(new Node);
    }

    // функция сравнения 2-х пар по частоте
    auto cmp = [](const tuple_set& a, const tuple_set& b) {
        return std::get<1>(a) < std::get<1>(b);
    };

    // сортируем по возрастанию
    sort(
        counter.begin(),
        counter.end(),
        cmp);

    size_t ind = 0;  // первый неиспользованный элемент
    size_t ind_sum = 0; // первая доступная сумма
    size_t ind_free = 0; // куда складываем сумму
    int added = 0; // сколько добавили в ind_free

    set src_set;  // множество символов
    int src_freq;  // и частота узла-потомка
    set dst_set;  // множество символов
    int dst_freq;  // и частота его предка

    // делаем проход по вектору с исходными символами
    while (ind < counter.size()) {
        // будущий предок узла. звучит немного странно)
        std::tie(dst_set, dst_freq) = counter_sum[ind_free];
        // частота текущего элемента с исходным символом
        int freq_c = std::get<1>(counter[ind]);
        // частота текущего элемента с суммами частот
        int freq_s = std::get<1>(counter_sum[ind_sum]);
        // если приемник и текущий элемент с суммой совпадают или у исходного меньше частота
        if (freq_c <= freq_s || ind_sum == ind_free) {
            std::tie(src_set, src_freq) = counter[ind];
            //значение символа
            int value = END_OF_SEQ;
            if (!src_set.empty()) {
                value = *src_set.begin();
            }
            // мержим в приемник
            dst_set.insert(src_set.begin(), src_set.end());
            dst_freq += src_freq;
            // и делаем его родителем исходного узла
            nodes_counter_sum[ind_free]->childs[added] = nodes_counter[value];
            // переходим к следующему исходному элементу
            ind++;
        } else {
            std::tie(src_set, src_freq) = counter_sum[ind_sum];
            // мержим в приемник сумму узлов
            dst_set.insert(src_set.begin(), src_set.end());
            dst_freq += src_freq;
            // и делаем его родителем этой суммы
            nodes_counter_sum[ind_free]->childs[added] = nodes_counter_sum[ind_sum];
            // эта сумма уже задействована
            ind_sum++;
        }
        added++;
        counter_sum[ind_free] = std::make_tuple(dst_set, dst_freq);
        // в приемник нужно положить 2 узла (левый 0, правый 1)
        if (added == 2) {
            ind_free++;
            added = 0;
        }
    }

    // исходные элементы кончились, делаем проход по оставшимся суммам
    // элементов было n, каждый раз мы из 2-х делали 1 и сохраняли их сумму
    // поэтому всего сумм будет n-1
    while (ind_free != counter.size() - 1) {
        // это приемник
        std::tie(dst_set, dst_freq) = counter_sum[ind_free];
        // сумма, которую еще не сложили
        std::tie(src_set, src_freq) = counter_sum[ind_sum];
        // мержим в приемник
        dst_set.insert(src_set.begin(), src_set.end());
        dst_freq += src_freq;
        // приходится перезаписывать. можно было взять Pair, но язык все равно поломать можно
        counter_sum[ind_free] = std::make_tuple(dst_set, dst_freq);
        // делаем приемник предком добавленного элемента
        nodes_counter_sum[ind_free]->childs[added] = nodes_counter_sum[ind_sum];
        // переходим к следующему
        ind_sum++;
        added++;
        // если есть левый и правый потомок
        if (added == 2) {
            ind_free++;
            added = 0;
        }
    }

    size_t last = nodes_counter_sum.size() - 1;
    // последний пустой узел можно удалить
    // об остальных позаботится дерево (вектор хранит указатели)
    delete nodes_counter_sum[last];
    if (last) {
        last--;
    }
    Node* root = nodes_counter_sum[last];

    Tree* tree = new Tree(root);

    return tree;
}

//============== end of Encoder

//============== Decoder

// public high-level method
// декодирует поток compressed в original
void Decoder::decode(IInputStream* compressed, IOutputStream* original) {
    BitSequence seq_tree_size = read_seq(compressed, MAX_TREE_BYTE_LEN);
    size_t tree_size = seq_tree_size.seq2size();
    BitSequence seq_tree = read_seq(compressed, tree_size);
    Tree tree;
    tree.deserialize(seq_tree);
    BitSequence seq = read_seq(compressed, MAX_PROBE);
    // yield?
    list_byte decoded = tree.decode(seq);
    for (auto it: decoded) {
        write(original, it);
    }
}

//============== end of Decoder

//============== Tree

//destruct in post-order traversal
Tree::~Tree() {
    vector<tuple_node> stack;
    tuple_node t = std::make_tuple(false, root);
    Node* node;
    bool visited;
    stack.push_back(t);
    while (!stack.empty()) {
        std::tie(visited, node) = stack.back();
        stack.pop_back();
        if (visited) {
            delete node;
            continue;
        }
        t = std::make_tuple(true, node);
        stack.push_back(t);
        if (node->childs[1]) {
            t = std::make_tuple(false, node->childs[1]);
            stack.push_back(t);
        }
        if (node->childs[0]) {
            t = std::make_tuple(false, node->childs[0]);
            stack.push_back(t);
        }
    }
}

// декодирует последовательность бит в байты
list_byte Tree::decode(const BitSequence& seq) {
    list_byte res;
    Node* node = root;
    for (size_t pos = 0; pos < seq.size(); pos++) {
        assert(node);
        if (node->terminal) {
            if (node->value == END_OF_SEQ) {
                break;
            }
            res.push_back(node->value);
            node = root;
        }
        int bit = seq.get_bit(pos);
        node = node->childs[bit];
    }
    return std::move(res);
}

// возвращает словарь символ -> последовательность бит для кодирования
// обход в глубину
dict_seq Tree::get_map() {
    dict_seq mymap;
    BitSequence seq;
    Node* node;

    using tuple_seq_node = std::tuple<BitSequence, Node*>;
    vector<tuple_seq_node> stack;
    tuple_seq_node t = std::make_tuple(seq, root);
    stack.push_back(t);

    while (!stack.empty()) {
        std::tie(seq, node) = stack.back();
        stack.pop_back();

        if (!node) {
            continue;
        }
        if (node->terminal) {
            mymap[node->value] = seq;
            continue;
        }
        for (int i=0; i < 2; i++) {
            BitSequence tmp = seq;
            tmp.append_bit(i);
            t = std::make_tuple(tmp, node->childs[i]);
            stack.push_back(t);
        }
    }
    return std::move(mymap);
}

// сохраняет дерево в последовательность бит
/* формат:
    [8 байт длина пути]
    [8 байт длина алфавита]
    [8 байт индекс END_POS_SEQ в алфавите]
    [путь] - 1 элемент 1 байт
    [алфавит] - 1 элемент 1 байт
*/
BitSequence Tree::serialize() {
    BitSequence seq;

    using tuple_byte_node = std::tuple<bool, byte, Node*>;
    vector<tuple_byte_node> stack;
    tuple_byte_node t = std::make_tuple(false, 'U', root);
    stack.push_back(t);
    Node* node;
    bool visited;
    byte c;

    list_byte path;
    list_byte values;
    size_t pos_end;

    while (!stack.empty()) {
        std::tie(visited, c, node) = stack.back();
        stack.pop_back();

        if (!node) {
            continue;
        }
        path.push_back(c);
        if (node->terminal) {
            values.push_back(node->value);
            if (node->value == -1) {
                pos_end = values.size() - 1;
            }
            continue;
        }
        if (visited) {
            continue;
        }
        t = std::make_tuple(true, 'U', node);
        stack.push_back(t);
        t = std::make_tuple(false, 'R', node->childs[1]);
        stack.push_back(t);
        t = std::make_tuple(true, 'U', node);
        stack.push_back(t);
        t = std::make_tuple(false, 'L', node->childs[0]);
        stack.push_back(t);
    }

    BitSequence path_size(path.size());
    BitSequence values_size(values.size());
    BitSequence pos_end_seq(pos_end);
    seq.append(path_size);
    seq.append(values_size);
    seq.append(pos_end_seq);
    seq.append(path);
    seq.append(values);

    return std::move(seq);
}

// строит дерево из последовательности бит
void Tree::deserialize(const BitSequence& seq) {
    BitSequence* tmp = new BitSequence;
    size_t offset = 0;
    list_byte raw_data;

    raw_data = seq.get_bytes(offset, MAX_TREE_BYTE_LEN);
    size_t path_size = tmp->append(raw_data)->seq2size();
    delete tmp;
    offset += MAX_TREE_BYTE_LEN;

    raw_data = seq.get_bytes(offset, MAX_TREE_BYTE_LEN);
    tmp = new BitSequence;
    size_t values_size = tmp->append(raw_data)->seq2size();
    delete tmp;
    offset += MAX_TREE_BYTE_LEN;

    raw_data = seq.get_bytes(offset, MAX_TREE_BYTE_LEN);
    tmp = new BitSequence;
    size_t pos_end = tmp->append(raw_data)->seq2size();
    delete tmp;
    offset += MAX_TREE_BYTE_LEN;

    list_byte path = seq.get_bytes(offset, path_size);
    offset += path_size;
    list_byte values = seq.get_bytes(offset, values_size);

    root = new Node;
    Node* node = nullptr;
    byte prev = 0;
    size_t ind = 0;
    vector<Node*> stack;
    stack.push_back(root);
    for (byte c: path) {
        switch (c) {
            case 'U':
                if (prev == 'L' || prev == 'R') {
                    if (node) {
                        node->terminal = true;
                        node->value = values[ind];
                        if (ind++ == pos_end) {
                            node->value = -1;
                        }
                    }
                }
                node = stack.back();
                stack.pop_back();
                break;
            case 'L':
                node->childs[0] = new Node;
                stack.push_back(node);
                node = node->childs[0];
                break;
            case 'R':
                node->childs[1] = new Node;
                stack.push_back(node);
                node = node->childs[1];
                break;
            default:
                throw "Unexpected path symbol.";
        }
        prev = c;
    }
}

//============== end of Tree

//============== BitSequence

// добавляет к последовательности 1 бит
BitSequence* BitSequence::append_bit(bool b) {
    // сколько осталось бит до полного байта
    size_t res = residual();
    if (res) {
        byte last = bits.back() | (b << (res - 1));
        bits.pop_back();
        bits.push_back(last);
    } else {
        byte added = b << (BYTE_SIZE - 1);
        bits.push_back(added);
    }
    bits_size++;
    return this;
}

// добавляет к последовательности бит 1 байт (BYTE_SIZE бит)
BitSequence* BitSequence::append(const byte& b) {
    // сколько осталось бит до полного байта
    size_t res = residual();
    // младшие (BYTE_SIZE - res) бит
    byte mask = (1 << (BYTE_SIZE - res)) - 1;
    if (res) {
        // добавляем res бит к последнему байту
        byte last = bits.back() | (b >> (BYTE_SIZE - res));
        // и остаток
        byte added = (b & mask) << res;
        bits.pop_back();
        bits.push_back(last);
        bits.push_back(added);
    } else {
        bits.push_back(b);
    }
    bits_size += BYTE_SIZE;
    return this;
}

// добавляет к последовательности бит другую последовательность
BitSequence* BitSequence::append(const BitSequence& seq) {
    if (!seq.size()) {
        return this;
    }
    if (!size()) {
        bits_size = seq.bits_size;
        bits = seq.bits;
        return this;
    }
    size_t full_bytes = seq.size() / BYTE_SIZE;
    // добавляем все полные байты
    for (size_t i = 0; i < full_bytes; i++) {
        append(seq.bits[i]);
    }
    // сколько бит осталось добавить
    size_t res = seq.size() % BYTE_SIZE;
    if (!res) {
        return this;
    }
    byte dst_last = bits.back();
    // достали последний байт
    bits.pop_back();
    // что осталось добавить
    byte last = seq.bits.back();
    last >>= BYTE_SIZE - res;
    // сколько бит можно добавить, не создавая новый байт
    size_t has_res = residual();
    if (has_res >= res) {
        // дополнительный байт не нужен, все вмещается
        last <<= has_res - res;

        dst_last |= last;
        bits.push_back(dst_last);
        bits_size += res;
    } else {
        // нужен дополнительный байт
        // какой хвост будет теперь
        size_t new_res = res - has_res;
        byte last1 = last >> new_res;
        byte mask = (1 << new_res) - 1;
        byte last2 = (last & mask) << (BYTE_SIZE - new_res);

        dst_last |= last1;
        bits.push_back(dst_last);
        bits.push_back(last2);
        bits_size += res;
    }
    return this;
}

// возвращает значение бита в позиции pos
// позиция 0 - самый старший бит
bool BitSequence::get_bit(size_t pos) const {
    assert(pos <= bits_size);
    byte b = bits[pos / BYTE_SIZE];
    //convert to big endian
    size_t more_than_bits = pos % BYTE_SIZE;
    more_than_bits = BYTE_SIZE - 1 - more_than_bits;
    byte mask = 1 << more_than_bits;
    return (b & mask);
}

// возвращает значение байта в позиции pos
byte BitSequence::get_byte(size_t pos) const {
    assert(pos <= size());
    return bits[pos];
}

// сколько бит не хватает до полного байта (0-7)
 const size_t BitSequence::complement(size_t size) {
    return (BYTE_SIZE - size % BYTE_SIZE) % BYTE_SIZE;
}

// преобразовывает последовательность в количество бит
// поскольку делается это ровно 1 раз, напишем понятно, но неэффективно
size_t BitSequence::seq2size() const {
    assert(size() <= MAX_TREE_BYTE_LEN * BYTE_SIZE);
    size_t res = 0;
    for (size_t pos=0; pos < size(); pos++) {
        size_t bit = static_cast<size_t>(get_bit(pos));
        res <<= 1;
        res |= bit;
    }
    return res;
}

// преобразовывает количество бит в последовательность
// поскольку делается это ровно 1 раз, напишем понятно, но неэффективно
BitSequence::BitSequence(size_t size):
    bits(),
    bits_size(0) {
    std::deque<byte> q;
    while (size) {
        q.push_front(size & 1);
        size >>= 1;
    }
    while (q.size() < MAX_TREE_BYTE_LEN * BYTE_SIZE) {
        q.push_front(0);
    }
    while (!q.empty()) {
        bool b = q.front();
        q.pop_front();
        append_bit(b);
    }
}

// добавляет к последовательности бит массив байт
BitSequence* BitSequence::append(const list_byte& b) {
    for (byte c: b) {
        append(c);
    }
    return this;
}

// возвращает nbytes байт, начиная с позиции offset
list_byte BitSequence::get_bytes(size_t offset, size_t nbytes) const {
    list_byte bytes(bits.begin() + offset, bits.begin() + offset + nbytes);
    return bytes;
}

//============== end of BitSequence
