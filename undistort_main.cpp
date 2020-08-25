#include "src/undistort.h"

using namespace std;

void help_print(string a){
    cout << a << " [calibResultPath] [Img] [outFile]" << endl;
}
int main(int argc, char **argv){
    
    if (argc < 3) {
        help_print(argv[0]);
        exit(-1);
    }
    string calibResultPath = argv[1];
    string srcImgPath = argv[2];
    char* file = NULL;
    if (argc >3) {
        file = argv[3];
    }
    
    CUndistort undistort(srcImgPath, calibResultPath);
    undistort.run(file);
    return 0;
}
