#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <print>
#include <span>
#include <sstream>
#include <vector>

struct Vec3{ float x=0, y=0, z=0; };

Vec3 operator+(const Vec3& lhs, const Vec3& rhs){
    return {lhs.x+rhs.x, lhs.y+rhs.y, lhs.z+rhs.z};
}

Vec3 operator*(float f, const Vec3& v){
    return {f*v.x, f*v.y, f*v.z};
}

Vec3 operator*(const Vec3& v, float f){
    return f*v;
}

Vec3 operator/(const Vec3& v, float f){
    return {v.x/f, v.y/f, v.z/f};
}

Vec3& operator+=(Vec3& lhs, const Vec3& rhs){
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    return lhs;
}

using Pos = Vec3;
using dPos = Vec3;

Pos applyDerives(Pos s, const dPos& d, const float h){
    s += d * h;

    return s;
}

dPos lorenz(const Pos& s, float sigma, float rho, float beta){
    return {
        .x = sigma*(s.y - s.x),
        .y = s.x*(rho - s.z) - s.y,
        .z = s.x*s.y - beta*s.z
    };
}

void rk4(Pos& s, float dl, float sigma, float rho, float beta){
    auto k1 = lorenz(s, sigma, rho, beta);
    auto k2 = lorenz(applyDerives(s, k1, 0.5*dl), sigma, rho, beta);
    auto k3 = lorenz(applyDerives(s, k2, 0.5*dl), sigma, rho, beta);
    auto k4 = lorenz(applyDerives(s, k3, dl), sigma, rho, beta);

    auto k = (k1 + 2*k2 + 2*k3 + k4)/6.0f;
    s += k * dl;
}

char toChar(uint8_t i){
    if(i==0)
        return ' ';

    return ".:-=*#%@"[i/32];
}

void submit(
    std::span<const uint8_t> buffer,
    const uint32_t width,
    const uint32_t height
){
    std::print("\033[H\033[?25l");

    std::stringstream ss;
    for(uint32_t i=0; i<height; ++i){
        for(uint32_t j=0; j<width; ++j)
            ss<<toChar(buffer[i*width + j]);
        ss<<'\n';
    }
    auto str = ss.str();

    std::print("{}", str);
}

struct Vec2{
    float x, y;

    Vec2(const Vec3& v)
        : x(v.x), y(v.z) {}
};

void drawPoint(const Vec2& p,
    std::span<uint8_t> buffer,
    const uint32_t width,
    const uint32_t height
){
    int x = (p.x+1)/2 * width, y = (-p.y+1)/2 * height;

    if(0 <= x && x < width && 0 <= y && y < height){
        buffer[y*width + x] = 255;
    }
}

void drawLine(const Vec2& p1, const Vec2& p2,
    std::span<uint8_t> buffer,
    const uint32_t width,
    const uint32_t height
){
    // Assume x, y of p1, p2 is in range [-1, 1]
    int x1 = ( p1.x+1)/2 *  width, x2 = ( p2.x+1)/2 *  width;
    int y1 = (-p1.y+1)/2 * height, y2 = (-p2.y+1)/2 * height;
    constexpr int pixelStep = 16;

    if(std::abs(x2-x1) >= std::abs(y2-y1)){
        if(x1 > x2){
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        else if(x1 == x2){
            if(0 <= x1 && 0 <= y1 && y1 < height){
                auto pos = y1*width + x1;
                buffer[pos] = std::min<uint8_t>(buffer[pos]+pixelStep, 255);
            }

            return;
        }

        for(int i=x1; i<=x2 && i<width; ++i){
            auto r = static_cast<float>(i-x1) / (x2-x1);
            int j = y1 + r*(y2-y1);

            if(0 <= i && 0 <= j && j < height){
                auto pos = j*width + i;
                buffer[pos] = std::min<uint8_t>(buffer[pos]+pixelStep, 255);
            }
        }
    }
    else{
        if(y1 > y2){
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        else if(y1 == y2){
            if(0 <= y1 && 0 <= x1 && x1 < width){
                auto pos = y1*width + x1;
                buffer[pos] = std::min<uint8_t>(buffer[pos]+pixelStep, 255);
            }

            return;
        }

        for(int i=y1; i<=y2 && i<height; ++i){
            auto r = static_cast<float>(i-y1) / (y2-y1);
            int j = x1 + r*(x2-x1);

            if(0 <= i && 0<= j && j < width){
                auto pos = i*width + j;
                buffer[pos] = std::min<uint8_t>(buffer[pos]+pixelStep, 255);
            }
        }
    }
}

void draw(std::span<const Pos> line,
    std::span<uint8_t> buffer, uint32_t width, uint32_t height
){
    // TODO. camera position and rotation
    // for now, assume camera in (0, 0, 0) and look forward(+z)

    for(int i=1; i<line.size(); ++i){
        // orthographic projection
        Vec2 o0 = line[i-1];
        Vec2 o1 = line[i];

        drawLine(o0, o1,
            buffer, width, height
        );
    }
}

using Clock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<float>;

volatile std::sig_atomic_t isRunning = 1;

void handler(int) {
    isRunning = 0;
}

int main(void){
    std::signal(SIGINT, handler);
    Pos s{0.5, 0.5, 0.5};

    std::vector<Pos> line;
    constexpr uint32_t width = 120, height = 40;
    std::array<uint8_t, width*height> buffer;

    Clock::time_point start = Clock::now();

    while(isRunning){
        Clock::time_point now = Clock::now();
        Seconds delta = now - start;

        auto dt = delta.count();

        if(dt > 1.0f/60){
            start = now;

            rk4(s, 1.0f/60, 10,28, 8.0f/3);
            line.push_back({
                .x = (s.x + 5) / 25,
                .y = (s.y +  0) / 25,
                .z = (s.z - 25) / 50
            });

            std::fill(buffer.begin(), buffer.end(), 0);

            draw(line, buffer, width, height);
            drawPoint(line.back(), buffer, width, height);
            submit(buffer, width, height);
        }
    }

    std::print("\033[?25h");
    return 0;
}