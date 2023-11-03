#include "../../include/interface/KeyBoard.h"
#include <iostream>

template<typename T>
inline T max(const T a, const T b){
	return (a > b ? a : b);
}

template<typename T>
inline T min(const T a, const T b){
	return (a < b ? a : b);
}

KeyBoard::KeyBoard(){
    userCmd = UserCommand::None;
    userValue.setZero();

    tcgetattr( fileno( stdin ), &_oldSettings );
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
}

KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
    switch (_c){
    // case ' ':
    //     return UserCommand::EXIT;
    case '1': 
        return UserCommand::Passive;//passive
    case '2': 
        return UserCommand::FixStand;//fixedStand
    case '4': 
        return UserCommand::Walking;// walking
    case ' ':
        userValue.setZero();
        return UserCommand::None;
    default:
        return UserCommand::None;
    }
}

void KeyBoard::changeValue(){
    switch (_c){
    case 'w':case 'W':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1);
        break;
    case 's':case 'S':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1);
        break;
    case 'd':case 'D':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1);
        break;
    case 'a':case 'A':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1);
        break;

    case 'i':case 'I':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1);
        break;
    case 'k':case 'K':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1);
        break;
    case 'l':case 'L':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1);
        break;
    case 'j':case 'J':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1);
        break;
    default:
        break;
    }
}

void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
}
template <typename T>
auto P(T const value)-> typename std::underlying_type<T>::type{
    return static_cast<typename std::underlying_type<T>::type>(value);
}
void* KeyBoard::run(void *arg){
    while(1){
        // std::cout << "key test" << std::endl;
        FD_ZERO(&set);
        FD_SET( fileno( stdin ), &set );

        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            read( fileno( stdin ), &_c, 1 );
            userCmd = checkCmd();
            std::cout<<"Key press: "<<P(userCmd)<<std::endl;
            if(userCmd == UserCommand::None)
                changeValue();
            _c = '\0';
        }
        usleep(1000);
    }
}