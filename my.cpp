#include "Huffman.h"

MyInputStream::MyInputStream() {
    freopen(NULL, "rb", stdin);
}

bool MyInputStream::Read(byte& value) {
    int c = getc(stdin);
    bool res = c != EOF;
    value = c;
    return res;
}

MyOutputStream::MyOutputStream() {
    freopen(NULL, "wb", stdout);
}

void MyOutputStream::Write(byte value) {
    putc(value, stdout);
}
