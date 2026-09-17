
#include "zf_global/util/call_back.h"
#include "gtest/gtest.h"

using namespace std;

BEGIN_NS_ZF

// Example callback functions
void MyCallback2(float arg1, int &arg2, const std::string &arg3)
{
    std::cout << "MyCallback2 invoked with arg1: " << arg1 << ", arg2: " << arg2 << ", arg3: " << arg3 << std::endl
              << std::endl;
}

void CallbackFunction(int a, float b)
{
    std::cout << "CallbackFunction called with: ";
    std::cout << "a type " << typeid(a).name();
    std::cout << ", b type " << typeid(b).name();
    std::cout << ", a = " << a << ", b = " << b << std::endl
              << std::endl;
}

struct A
{
    void Fun(int i) { cout << i << endl; }
    void Fun1(int i, double j) { cout << i + j << endl; }
    int sum(int i, int j, int k)
    {
        std::cout << "sumFn(1, 2, 3) : " << (i + j + k) << std::endl;
        return i + j + k;
    }
    void print()
    {
        // auto d = CreateDelegate(this, &A::Fun); // 创建委托
        // d(654);                                 // 调用委托，将输出1
        CallBackBase sync1;
        sync1.RegisterCallback(&A::Fun, this);
        sync1.InvokeCallback(634);
    }
};

END_NS_ZF
using namespace NS_ZF;

int main()
{
    float arg3 = 3.14;
    int arg4 = 123;
    std::string arg5 = "OpenAI";
    CallBackBase sync;
    auto cbT = sync.RegisterCallback(&CallbackFunction);
    cbT(1, 1.5);
    sync.InvokeCallback(2, 0.5f);

    A a;
    a.print();
    sync.RegisterCallback(&A::sum, &a);
    auto rsl = sync.InvokeCallback<int>(5, 8, 9);
    std::cout << int() << " A::sum : " << rsl << std::endl << std::endl;

    std::function<void(int, float)> callback = std::bind(&CallbackFunction, std::placeholders::_1, std::placeholders::_2);
    sync.RegisterCallback(callback);
    sync.InvokeCallback(2, 2.5f);

    // Register the callback function using std::bind
    Delegate<void, float, int &, const std::string &> handler;
    handler.RegisterCallback(std::bind(&MyCallback2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    handler.InvokeCallback(arg3, arg4, arg5);

    auto cb = CreateDelegate(&CallbackFunction);
    cb(55, 73.4);
    return 0;
}