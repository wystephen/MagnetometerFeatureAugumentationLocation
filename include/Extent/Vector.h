#include "matrix.h"


template<class T>
class Vector : public Matrix<T> {
public:
    Vector<T>() {}

    Vector<T>(int len) : Matrix<T>(1, len) {
    }

    ~Vector<T>() {

    }

protected:


private:


};