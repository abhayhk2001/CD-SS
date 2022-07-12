struct hello {
  int num;
  int hi(int abhay) { return abhay; }
};

class world {
  int n;

 public:
  int x;
  int hi(int name) { return name; }
};

void func1(hello h1, hello h2, world w1, world w2, int& s, int* a) {
  h1.num = 1;
  h1.num++;
  s = (char)h2.num;
  int d = h1.hi(10);
  w1.hi(4);
  w2.x = 3;
}