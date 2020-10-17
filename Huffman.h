#ifndef __HUFFMAN__
#define __HUFFMAN__ 1
#include <cstdio>

typedef unsigned char byte;

struct IInputStream {
	// Возвращает false, если поток закончился
	bool virtual Read(byte& value) = 0;
};

struct IOutputStream {
	void virtual Write(byte value) = 0;
};

// Метод архивирует данные из потока original
void Encode(IInputStream& original, IOutputStream& compressed);
// Метод восстанавливает оригинальные данные
void Decode(IInputStream& compressed, IOutputStream& original);

struct MyInputStream: public IInputStream {
	// Возвращает false, если поток закончился
    MyInputStream();
    bool Read(byte& value);
};

struct MyOutputStream: public IOutputStream {
    MyOutputStream();
	void Write(byte value);
};

#endif