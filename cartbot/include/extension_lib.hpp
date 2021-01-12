#ifndef EXTENSION_LIB_HPP
#define EXTENSION_LIB_HPP

#include <stdio.h>
#include <sys/stat.h>

#include <string>
#include <iostream>


bool checkExistence(std::string str)
{
    struct stat st;
    //0 : ファイル又はディレクトリの存在
    //0以外 : ファイル又はディレクトリの存在なし
    int res = stat(str.c_str(), &st);
    bool b_res = false;
    if (res == 0){
        b_res = true;
    }
    return b_res;
}

bool createFolder(std::string str)
{
    bool res = false;
    if (mkdir(str.c_str(),
                S_IRUSR | S_IWUSR | S_IXUSR |         /* rwx */
                S_IRGRP | S_IWGRP | S_IXGRP |         /* rwx */
                S_IROTH | S_IXOTH | S_IXOTH) == 0) {  /* rwx */
        std::cout << "create " << str << std::endl;
        res = true;
    }
    else {
        std::cout <<  "can't create " << str << std::endl;
        perror("mkdir() error");
        res = false;
    }

    return res;
}

#endif