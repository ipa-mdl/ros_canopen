#include <socketcan_interface/dummy.h>
#include <socketcan_interface/threading.h>
#include <canopen_master/canopen.h>

// Bring in gtest
#include <gtest/gtest.h>

canopen::ObjectDictSharedPtr  make_dict(){
    canopen::DeviceInfo info;
    info.nr_of_rx_pdo = 0;
    info.nr_of_tx_pdo = 0;

    canopen::ObjectDictSharedPtr  dict = std::make_shared<canopen::ObjectDict>(info);
    dict->insert(false, std::make_shared<const canopen::ObjectDict::Entry>(canopen::ObjectDict::VAR,
                                                                           0x1017,
                                                                           canopen::ObjectDict::DEFTYPE_UNSIGNED16,
                                                                           "producer heartbeat",
                                                                           true, true, false,
                                                                           canopen::HoldAny((uint16_t)0),
                                                                           canopen::HoldAny((uint16_t)100)
                                                                           ));
    return dict;
}
TEST(TestNode, testInitandShutdown){

    can::ThreadedDummyInterfaceSharedPtr driver = std::make_shared<can::ThreadedDummyInterface>();

    driver->add("0#8201", "701#00" ,false);
    driver->add("0#0101", "701#05" ,false);
    driver->add("601#2b17100000000000", "581#6017100000000000" ,false);
    driver->add("601#2b17100064000000", "581#6017100000000000" ,false);

    auto settings = can::SettingsMap::create();
    settings->set("trace", true);

    driver->init("unused", false, settings);

    canopen::NodeSharedPtr node = std::make_shared<canopen::Node>(driver, make_dict(), 1);

    {
        canopen::LayerStatus status;
        node->init(status);
        ASSERT_TRUE(status.bounded<canopen::LayerStatus::Ok>());
        ASSERT_EQ(canopen::Node::Operational, node->getState());
    }

    {
        canopen::LayerStatus status;
        node->shutdown(status);
        ASSERT_TRUE(status.bounded<canopen::LayerStatus::Ok>());
    }
    driver->shutdown();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
