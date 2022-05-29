#include <iostream>
using namespace std;
int main() {
    std::cout << "Hello, World!" << std::endl;

    string laserFile = "../data/0000000000.bin";
    //allocate 4MB buffer (around ~130 * 4 * 4 KB)
    int32_t num = 1000000;
    float *data = (float *) malloc(num * sizeof(float));
    //pointers for reading laser point
    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;

    //load point cloud
    FILE *stream;
    stream = fopen(laserFile.c_str(), "rb");
    num = fread(data, sizeof(float), num, stream) / 4;
    int i = 0;
    for (int32_t i=0;i<num;i++)
    {
        cout<<"Point "<<i<<" : "<<*px<<" "<<*py<<" "<<*pz<<" "<<*pr<<endl;
        px+=4;py+=4;pz+=4;pr+=4;
    }
    cout<<"total point num "<<num<<endl;
    fclose(stream);
    return 0;
}
