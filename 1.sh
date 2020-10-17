#!/usr/bin/env bash

./huffman_encode <in.txt >test
./huffman_decode <test >test_decoded
diff <(xxd in.txt) <(xxd test_decoded)

# В качестве данных для сжатия выступают:
# 4 текстовых документа, 1 картинка в формате bmp, 1 картинка в формате jpg.
