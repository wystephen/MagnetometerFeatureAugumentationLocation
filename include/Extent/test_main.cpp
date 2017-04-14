#include <iostream>

#include <thread>
#include <algorithm>


#include <random>

#include "FileReader.h"
#include "JsonCodert.h"
#include "JsonObject.h"
#include "CSVReader.h"

#include "time_stamp.h"

int main() {
    //JsonObject job;
    //JsonObject t = job["aaa"];

//	CSVReader fr(std::string("u1.csv"));

    CSVReader fr(std::string("Zupt.data.csv"));
//	fr.test1();
    for (int i(0); i < fr.rows_; ++i) {
        std::cout << *fr.GetMatrix()(i, 0) << std::endl;
    }

    //CSVReader magic(std::string("magicmatrix.csv"));
    //Matrix<double> src_m(magic.GetMatrix());

    //CSVReader inv(std::string("pinv.csv"));
    //Matrix<double> inv_m(inv.GetMatrix());



    printf("%20.20f", TimeStamp::now());

    getchar();

    return 0;

}
