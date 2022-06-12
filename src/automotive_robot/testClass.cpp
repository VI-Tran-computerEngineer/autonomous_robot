#include <iostream>

using namespace std;

class B{
    private:
    int* a;
    char* b;
    public:
    void setField(int* a, char*b){
        this->a = a;
        this->b = b;
    }
    void printField(){
        cout << "a's poiter: " << *a <<"    b's pointer: " << *b << endl;
    }
};

class A{
    private:
    int a;
    char b;
    B bb;
    public:
    A(int a, char b, B bb){
        this->a = a;
        this->b = b;
        this->bb = bb;
    }
    void printField(){
        cout << "&a: " << a << "  &b: " << b << endl;// << " B: B.a=" << bb.a << " B.b =" << bb.b << endl;
        this->bb.setField(&this->a, &this->b);
        this->bb.printField();
    }
};
int main(){
    B testB;
    A testA(14, 't', testB);
    testA.printField();
}
