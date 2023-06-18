#ifndef KEYSTROKES_HPP
#define KEYSTROKES_HPP


#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include <iostream>


struct termios orig_termios;

void reset_terminal_mode() {
    tcsetattr(0, TCSANOW, &orig_termios);
}

class KeyStrokes {
    public:
    KeyStrokes() { this->set_conio_terminal_mode(); }
    ~KeyStrokes() {}



    void set_conio_terminal_mode() {
        struct termios new_termios;

        /* take two copies - one for now, one for later */
        tcgetattr(0, &orig_termios);
        memcpy(&new_termios, &orig_termios, sizeof(new_termios));
        

        /* register cleanup handler, and set the new terminal mode */
        atexit(reset_terminal_mode);
        cfmakeraw(&new_termios);
        tcsetattr(0, TCSANOW, &new_termios);
    }


    int kbhit() {
        struct timeval tv = { 0L, 0L };
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        int result = select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
        if (result == -1) {
            return 0;
        }
        return FD_ISSET(STDIN_FILENO, &fds);
    }

    char m() {

        if (kbhit()) {
            int c = getchar();
            std::cout << c << std::endl;

            switch (c)
            {
            case 's':
                return 's'; // start
            case 'p':
                return 'p'; // posehold mode
            case 'o':
                return 'o'; // stabilize mode
            case 'q':
                return 'q'; // quite
            default:
                return 'g'; // good
            }
        }

        return 'g'; // good
    }

};

#endif // KEYSTROKES