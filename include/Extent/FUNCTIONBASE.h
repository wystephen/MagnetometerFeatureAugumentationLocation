#pragma once

template<class ValueType, class FuncValueType, class DrivativeValueType, int VecLen>
class FUNCTIONBASE {
public:
    FUNCTIONBASE() {}

    /*
    This function will return a value according to the function value.
    */
    virtual FuncValueType operator()(ValueType val) = 0;

    /*
    Compute the drivative value of the function at the point represent by val.
    */
    virtual DrivativeValueType derivative(ValueType val) = 0;

    /*
    A shorter function name for derivative.
    */
    DrivativeValueType d(ValueType val) {
        return derivative(val);
    }

protected:

private:


};