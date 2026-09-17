
#include "factory.h"
#include "gtest/gtest.h"
#include <rclcpp/rclcpp.hpp>

BEGIN_NS_ZF

class Base
{
public:
    virtual std::string Name() const { return "base"; }
};

class Derived : public Base
{
public:
    virtual std::string Name() const { return m_name; }
    Derived(){};
    Derived(const std::string &name)
    {
        m_name = name;
        LOG_INFO() << "m_name Derived " << m_name;
    }
    // Derived(){LOG_SCREEN() << "m_name aa "<<m_name; m_name = "derived";}
private:
    std::string m_name;
};

TEST(FactoryTest, Register)
{
    Factory<std::string, Base,
            Base *(*)(const std::string &node)>
        factory;
    EXPECT_TRUE(factory.Register("derived_class",
                                 [](const std::string &node) -> Base *
                                 {
                                     return new Derived(node);
                                 }));

    auto derived_ptr = factory.CreateObject("derived_class", "customs_aaa");
    EXPECT_NE(nullptr, derived_ptr);
    EXPECT_EQ("derived", derived_ptr->Name());
    LOG_INFO() << "Name: " << derived_ptr->Name(); 
    auto non_exist_ptr = factory.CreateObject("non_exist_class", "customs_aaa");
    EXPECT_EQ(nullptr, non_exist_ptr);
}

END_NS_ZF

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
    return result;
}