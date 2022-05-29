#include <iostream>
#include <fstream>
#include <vector>

using namespace std;
int main()
{
    string vstrScanFilename = "0000000007.bin";
    vector<vector<double>> laserPoints;
    int32_t num = 1000000;
    for(int i = 0; i<num; i++)
    {
        vector<double> point = {0,0,0,0};
        laserPoints.push_back(point);
    }
    //allocate 4MB buffer (around ~130 * 4 * 4 KB)
    float *data = (float *) malloc(num * sizeof(float));
    //pointers for reading laser point
    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;

    //load point cloud
    FILE *fstream;
    fstream = fopen(vstrScanFilename.c_str(), "rb");
    num = fread(data, sizeof(float), num, fstream)/4;
    for(int i=0; i<num;i++)
    {
        laserPoints[i][0] = *px;
        laserPoints[i][1] = *py;
        laserPoints[i][2] = *pz;
        laserPoints[i][3] = *pr;
        px+=4;py+=4;pz+=4;pr+=4;
    }
    fclose(fstream);
    //reset laserpoint vector size
    laserPoints.resize(num);

    //write ply before undistort
    ofstream outfile("scans.ply",std::ofstream::binary);
    outfile<<"ply"<<endl;
    outfile<<"format ascii 1.0"<<endl;
    outfile<<"element vertex "<<num<<endl;
    outfile<<"property float x"<<endl;
    outfile<<"property float y"<<endl;
    outfile<<"property float z"<<endl;
    outfile<<"end_header"<<endl;
    //write to ply file
    for(int i =0; i<num;i++)
    {
        outfile<<laserPoints[i][0]<<" "<<laserPoints[i][1]<<" "<<laserPoints[i][2]<<endl;
    }
    outfile.close();


    return 0;
}
