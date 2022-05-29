#include <iostream>
#include <sstream>
#include <fstream>
using namespace std;
int main() {
    std::cout << "Hello, World!" << std::endl;
    string strFile = "/home/xin/Downloads/DATASET/TUM/rgbd_dataset_freiburg3_long_office_household/txtfile.txt";
    cout<<"enter loadimages function"<<endl;
    cout<<"file str "<<strFile.c_str()<<endl;
    ifstream f;
    f.open(strFile.c_str());
    if(f.fail())
        cout<<"open txt fail"<<endl;
    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);
    cout<<"check point first 3 lines"<<s0<<endl;
    int counter = 0;
    while(!f.eof())
    {
        string s;
        getline(f,s);
        cout<<"counter "<<counter<<" line "<<s<<endl;
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            //vTimestamps.push_back(t);
            ss >> sRGB;
            //vstrImageFilenames.push_back(sRGB);
        }
        counter++;
    }

    return 0;
}
