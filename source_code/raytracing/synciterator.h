#ifndef synciterator_h
#define synciterator_h

#include <mutex>

class synciterator {
private:
    std::mutex m;
    int i, j, x, y;
public:
    synciterator(int to_x, int to_y);
    std::pair<int, int> next();
};

#endif // !synciterator_h


