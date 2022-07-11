#include <set>
#include <vector>
#include <string>

void func2(std::set<int> set1, std::vector<int> arr, std::string s)  {
    arr.push_back(3);
    set1.insert(1);
    arr[0] = 1;
    s = "abhay";
}