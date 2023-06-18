#include <mutex>
#include <iostream>
#include <curses.h>
#include <thread>
#include <cstdio>

using namespace std;

class Derived {
    public:
    Derived() {cout << "derived constructor\n";};
    ~Derived() {
        cout << "derived destructor\n";
    }

    virtual void func1() {cout << "base func 1\n"; this->func2();}

    void func2() {cout << "derived func2 \n";};

};

class Base {
    public:
    Base() {
        cout << "base constructor\n";

        D = new Derived();

    };
    ~Base() {
        cout << "base destructor\n";
        delete D;
    }

    virtual void func1() {cout << "base func 1\n";}

    private:
    Derived* D;
};


void getchy() {
    cout << "mason\n";
    // initscr();

    // while (true) {
    //     addstr("hit a key: \n");

    //     int ch = getch();


    //     if (ch == 'q') {
    //         break;
    //     }
    // }


    // endwin();
}

char sharedVal = 0;
std::mutex mtx;

void getchy2(int id) {
    // initscr();

    while (true) {

        mtx.lock();

        char c = getch();

        if (id == 0) {
            sharedVal = c;
        }

        mtx.unlock();

        std::cout << c << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    // endwin();
};



int main() {
    // Base b = Base();

    std::thread t0[2];

    initscr();
    cbreak();
    noecho();

    t0[0] = std::thread(getchy2, 0);
    t0[1] = std::thread(getchy2, 1);

    for (int i=0; i<2; i++) {
        t0[i].join();
    }

    std::cout << "exiting\n";

    return 0;
}