#include <iostream>
#include <thread>

using namespace std;

static int num = 0;

void run() {
  for (int i = 0; i < 100000; i++) {
    num++;
    printf("num = %d\n", num);
  }
  cout << "end" << endl;
}

int main() {
  thread th(run);
  for (int i = 0; i < 100; i++) {
    printf("i = %d\n", i);
  }
  th.detach();
  return 0;
}
