#include "uart.hpp"
#include <iostream>
#include <bitset>

void UART_RX::put_samples(const unsigned int *buffer, unsigned int n)
{

    for (int i = 0; i < n; i++) {
        if (state == State::Espera) {
            if (buffer[i] != 1) {
                state = State::ComecoRecebimento;
                sample_count++;
            }
        } else if (state == State::ComecoRecebimento) {
            if (sample_count < (SAMPLES_PER_SYMBOL / 2 + (SAMPLES_PER_SYMBOL * 3) / 32)) {
                sample_count++;
            } else {
                int last_bit = check_bits();
                last_bits.clear();
                
                if (last_bit == -1 | last_bit == 1)
                    state = State::Espera;
                else
                    state = State::Recebimento;

                sample_count = 0;
                continue;
            }
        } else if (state == State::Recebimento) {
            if (bits_count < 7) {
                if (sample_count < SAMPLES_PER_SYMBOL-1) {
                    sample_count++;
                }
                else
                {
                    sample_count = 0;
                    int last_bit = check_bits();
                    last_bits.clear();
                    if (last_bit == -1) {
                        state = State::Espera;
                        sample_count = 0;
                        continue;
                    }
                    byte = byte | (last_bit << bits_count);
                    bits_count++;
                }
            } else {
                if (sample_count < SAMPLES_PER_SYMBOL - 1) {
                    sample_count++;
                } else {
                    int last_bit = check_bits();
                    last_bits.clear();
                    if (last_bit == -1) {
                        state = State::Espera;
                        sample_count = 0;
                        continue;
                    }
                    byte = byte | (last_bit << bits_count);
                    byte_counter++;
                    std::bitset<8> bit_byte(byte);
                    get_byte(byte);
                    byte = 0;
                    sample_count = 2;
                    bits_count = 0;
                    state = State::FimRecebimento;
                }
            }
        } else if (state == State::FimRecebimento) {
            if (sample_count < (SAMPLES_PER_SYMBOL / 2)) {
                sample_count++;
            } else {
                int last_bit = check_bits();
                last_bits.clear();
                sample_count = 0;
                state = State::Espera;
            }
        }
        last_bits.push_back(buffer[i]);
    }
}

int UART_RX::check_bits() {
    int ones = 0;
    int zeroes = 0;
    if (last_bits.size() < 30)
        return -1;

    for (int i = last_bits.size() - 31; i < last_bits.size(); i++) {
        if (last_bits[i] == 1)
            ones++;
        else
            zeroes++;
    }
    if (zeroes > 25)
        return 0;
    if (ones > 25)
        return 1;
    return -1;
}

void UART_TX::put_byte(uint8_t byte)
{
    samples_mutex.lock();
    put_bit(0);
    for (int i = 0; i < 8; i++) {
        put_bit(byte & 1);
        byte >>= 1;
    }
    put_bit(1);
    samples_mutex.unlock();
}

void UART_TX::get_samples(unsigned int *buffer, unsigned int n) {
    samples_mutex.lock();
    std::vector<unsigned int>::size_type i = 0;
    while (!samples.empty() && i < n) {
        buffer[i++] = samples.front();
        samples.pop_front();
    }
    samples_mutex.unlock();

    while (i < n) buffer[i++] = 1;
}

void UART_TX::put_bit(unsigned int bit) {
    for (int i = 0; i < SAMPLES_PER_SYMBOL; i++)
        samples.push_back(bit);
}