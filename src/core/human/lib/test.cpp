#include "KeyStrokes.hpp"


int main(int argc, char** argv) {
    KeyStrokes k = KeyStrokes();

    while (1) {
        int bb = k.m();
        if (bb) {
            break;
        }
    }

    return 0;
}