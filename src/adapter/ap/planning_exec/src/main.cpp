/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: main代码模板
 */
#include <iostream>
#include "applicationswcexecutable2_manager.h"

int32_t main(int32_t argc, char* argv[])
{
    (void)argc;
    (void)argv;
    try {
        mdc::applicationswcexecutable2::Applicationswcexecutable2Manager manager;
        auto res =  manager.Execute();
        return res;
    } catch (const std::exception &exc) {
        std::cerr << "Applicationswcexecutable2 process catch an exception: " << exc.what() << &std::endl;
    } catch (...) {
        std::cerr << "Applicationswcexecutable2 process catch an unknown exception" << &std::endl;
    }
    return 0;
}