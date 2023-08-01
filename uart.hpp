#ifndef UART_HPP
#define UART_HPP

#include <functional>
#include <deque>
#include <mutex>
#include <stdint.h>
#include "config.hpp"
#include <cmath>

class UART_RX
{
public:
    UART_RX(std::function<void(uint8_t)> get_byte) : get_byte(get_byte) {inFrame = 0;}
    ~UART_RX() {while(inFrame++ < 9) {byteAtual.push_back(0);} get_byte(conversor(byteAtual));}
    void put_samples(const unsigned int *buffer, unsigned int n);
    uint8_t conversor(std::vector<unsigned int> bitAtual){ int valorfinal = 0; for(int i = 0;i<8;i++){ valorfinal += (bitAtual[i]) ? pow(2,7-i) : 0; }
    // std::cout<<valorfinal<< " ";
    return (uint8_t) valorfinal;
}
private:
    std::vector<unsigned int> classBuffer = {};
    std::vector<unsigned int> byteAtual = {};
    unsigned int inFrame;
    std::function<void(uint8_t)> get_byte;
};

class UART_TX
{
public:
    void put_byte(uint8_t byte);
    void get_samples(unsigned int *buffer, unsigned int n);
private:
    std::deque<unsigned int> samples;
    std::mutex samples_mutex;
    void put_bit(unsigned int bit);
};

#endif
