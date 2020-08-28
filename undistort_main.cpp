#include "src/undistort.h"

using namespace std;

void help_print(string a){
    cout << a << " [calibResultFile] [Img] [outFile]" << endl;
}
int main(int argc, char **argv){
    
    if (argc < 3) {
        help_print(argv[0]);
        exit(-1);
    }
    string calibResultFile = argv[1];
    string srcImgPath = argv[2];
    char* file = NULL;
    if (argc >3) {
        file = argv[3];
    }
    
    CUndistort undistort(srcImgPath, 0, calibResultFile);
    undistort.run(file);
    return 0;
}
