#include "uart.hpp"
#include <iostream>
#include "math.h"
#include <bits/stdc++.h>

// def put_samples(self, data):
//     fs = self.fs
//     T = 1/fs
//     SIZE = fs//300
//     omega0, omega1 = self.rx_omega0, self.rx_omega1
//     r = 0.99
//     rr = 0.9999

//     s = np.concatenate((self.rxbuf, data))
//     self.rxbuf = s[-SIZE:]
//     s = s[-SIZE-len(data):]

//     v0rp, v0ip, v1rp, v1ip = self.v0rp, self.v0ip, self.v1rp, self.v1p
//     vp, vpp = self.vp, self.vpp
//     yp = self.yp

//     rL = r**SIZE
//     c0LT, s0LT, c0T, s0T = np.cos(omega0*SIZE*T), np.sin(omega0*SIZE*T), np.cos(omega0*T), np.sin(omega0*T)
//     c1LT, s1LT, c1T, s1T = np.cos(omega1*SIZE*T), np.sin(omega1*SIZE*T), np.cos(omega1*T), np.sin(omega1*T)
//     cfb = np.cos(2*np.pi*300/fs)

//     for n in range(SIZE, len(s)):
//         v0r = s[n] - rL*c0LT*s[n-SIZE] + r*c0T*v0rp - r*s0T*v0ip
//         v0i = -rL*s0LT*s[n-SIZE] + r*c0T*v0ip + r*s0T*v0rp
//         v1r = s[n] - rL*c1LT*s[n-SIZE] + r*c1T*v1rp - r*s1T*v1ip
//         v1i = -rL*s1LT*s[n-SIZE] + r*c1T*v1ip + r*s1T*v1rp
//         v0rp, v0ip, v1rp, v1ip = v0r, v0i, v1r, v1i

//         v0 = v0r*v0r + v0i*v0i
//         v1 = v1r*v1r + v1i*v1i

//         # carrier detection
//         rho = v0 + v1
//         #print('%20.8f\r'% rho)
//         if self.rxcd <= 0 and rho > 20:
//             self.rxcd = 600
//         elif self.rxcd > 0 and rho < 5:
//             self.rxcd -= 1

//         # clock recovery
//         c = abs(v1-v0)
//         v = (1-rr)*c + 2*rr*cfb*vp - rr*rr*vpp
//         y = v - vpp
//         vp, vpp = v, vp

//         if self.rxcd > 0:
//             # clock signal above threshold
//             if y < -0.03:
//                 self.rxclkdist = -1
//             self.rxclkdist += 1
//             if self.rxclkdist < 50:
//                 # positive zero crossing
//                 if yp < 0 and y > 0:
//                     self.rxsamplerbuf = []
//             yp = y

//             if self.rxsamplerbuf is not None:
//                 self.rxsamplerbuf.append(v1-v0)
//                 if len(self.rxsamplerbuf) == 48000*46//self.fs:
//                     decision = np.dot(
//                         self.rxfilt, self.rxsamplerbuf[-len(self.rxfilt):])
//                     self.rxbits.append(1 if decision > 0 else 0)
//                     self.rxsamplerbuf = None
    
//     self.v0rp, self.v0ip, self.v1rp, self.v1p = v0rp, v0ip, v1rp, v1ip
//     self.vp, self.vpp = vp, vpp
//     self.yp = yp

// void UART_RX::put_samples(const unsigned int *buffer, unsigned int n) {
//     int fs = SAMPLING_RATE;
//     float T = SAMPLING_PERIOD;
//     int SIZE = SAMPLES_PER_SYMBOL;
//     float omega0 = 2*std::numbers::pi*(1080 + 100);
//     float omega1 = 2*std::numbers::pi*(1080 - 100);
//     float r = 0.99;
//     float rr = 0.9999;

//     int *s = {};


//     s = np.concatenate((self.rxbuf, data))
//     self.rxbuf = s[-SIZE:]  
//     s = s[-SIZE-len(data):]

//     v0rp, v0ip, v1rp, v1ip = self.v0rp, self.v0ip, self.v1rp, self.v1p
//     vp, vpp = self.vp, self.vpp
//     yp = self.yp

void UART_RX::put_samples(const unsigned int *buffer, unsigned int n) {
    // std::cout << "Adicionando " << n << " elementos ao buffer!\n";
    int SIZE = SAMPLES_PER_SYMBOL;

    std::vector<unsigned int> dest(buffer, buffer + n);
    classBuffer.insert(classBuffer.end(), dest.begin(), dest.end());
    while (classBuffer.size() >= SIZE) {
        std::vector<unsigned int> bitAtual(classBuffer.begin(), classBuffer.begin() + SIZE);
        if(inFrame != 0 && inFrame != 9) {
            // float acumulado = std::accumulate(bitAtual.begin() + 110 - 15, bitAtual.begin() + 110 + 15, 0) / 30;
            float acumulado = bitAtual[79];
            byteAtual.push_back(acumulado);
        }
        else if (inFrame == 9) {
            get_byte(conversor(byteAtual));
            byteAtual.erase(byteAtual.begin(), byteAtual.end());
        }
        inFrame = (inFrame + 1) % 10;
        classBuffer.erase(classBuffer.begin(), classBuffer.begin() + SIZE);
    }
}



void UART_TX::put_byte(uint8_t byte)
{
    samples_mutex.lock();
    put_bit(0);  // start bit
    for (int i = 0; i < 8; i++) {
        put_bit(byte & 1);
        byte >>= 1;
    }
    put_bit(1);  // stop bit
    samples_mutex.unlock();
}

void UART_TX::get_samples(unsigned int *buffer, unsigned int n)
{
    samples_mutex.lock();
    std::vector<unsigned int>::size_type i = 0;
    while (!samples.empty() && i < n) {
        buffer[i++] = samples.front();
        samples.pop_front();
    }
    samples_mutex.unlock();

    while (i < n) {
        // idle
        buffer[i++] = 1;
    }
}

void UART_TX::put_bit(unsigned int bit)
{
    for (int i = 0; i < SAMPLES_PER_SYMBOL; i++) {
        samples.push_back(bit);
    }
}
