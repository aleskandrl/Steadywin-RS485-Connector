// --- Файл: main.cpp ---
#include "steadywin/steadywin_motor.h"
#include "steadywin/windows_serial_port.h" // Assuming this file exists and implements the ISerialPort interface for Windows
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <conio.h> 
#include <windows.h>
#include <setupapi.h>
#include <devguid.h>
#include <atomic>
#include <future>
#include <mutex>
#include <sstream>
#include <deque>
#include <ctime>

#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "ws2_32.lib")

using namespace steadywin;

// =================================================================================
// UI Namespace - all rendering logic is encapsulated here
// =================================================================================
namespace ui {
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    std::mutex console_mutex;

    enum Color {
        BLACK = 0, BLUE = 1, GREEN = 2, CYAN = 3, RED = 4, MAGENTA = 5, YELLOW = 6, WHITE = 7,
        GRAY = 8, LIGHT_BLUE = 9, LIGHT_GREEN = 10, LIGHT_CYAN = 11, LIGHT_RED = 12, PINK = 13, LIGHT_YELLOW = 14, BRIGHT_WHITE = 15
    };

    void setColor(Color foreground, Color background = BLACK) {
        SetConsoleTextAttribute(hConsole, static_cast<WORD>(foreground | (background << 4)));
    }

    void setCursor(int x, int y) {
        SetConsoleCursorPosition(hConsole, { (SHORT)x, (SHORT)y });
    }

    void drawBox(int x, int y, int w, int h, const std::string& title) {
        std::lock_guard<std::mutex> lock(console_mutex);
        setColor(GRAY);
        setCursor(x, y); std::cout << (char)201;
        for (int i = 0; i < w - 2; ++i) std::cout << (char)205;
        std::cout << (char)187;

        for (int i = 1; i < h - 1; ++i) {
            setCursor(x, y + i); std::cout << (char)186;
            for (int j = 1; j < w - 1; ++j) std::cout << ' ';
            setCursor(x + w - 1, y + i); std::cout << (char)186;
        }

        setCursor(x, y + h - 1); std::cout << (char)200;
        for (int i = 0; i < w - 2; ++i) std::cout << (char)205;
        std::cout << (char)188;

        if (!title.empty()) {
            setCursor(x + 3, y);
            setColor(WHITE);
            std::cout << " " << title << " ";
        }
    }

    void clear() {
        CONSOLE_SCREEN_BUFFER_INFO csbi;
        GetConsoleScreenBufferInfo(hConsole, &csbi);
        DWORD count;
        DWORD cellCount = csbi.dwSize.X * csbi.dwSize.Y;
        FillConsoleOutputCharacter(hConsole, (TCHAR)' ', cellCount, { 0, 0 }, &count);
        FillConsoleOutputAttribute(hConsole, csbi.wAttributes, cellCount, { 0, 0 }, &count);
        setCursor(0, 0);
    }
    
    void drawLayout(const std::string& motor_port, uint8_t addr) {
        clear();
        setColor(BLACK, GRAY);
        setCursor(0, 0);
        std::cout << std::string(80, ' ');
        setCursor(2, 0);
        setColor(BLACK, GRAY);
        std::cout << " STEADYWIN MOTOR CONTROL TERMINAL v3.40 ";
        setCursor(55, 0);
        std::cout << "Port: " << std::left << std::setw(10) << motor_port << " Addr: " << (int)addr;

        drawBox(1, 2, 38, 10, "Telemetry");
        drawBox(41, 2, 38, 10, "Status");
        drawBox(1, 13, 78, 12, "Settings & Control");
        drawBox(1, 26, 78, 5, "Log");
        
        setColor(WHITE);
        setCursor(43, 4); std::cout << "Motor Enabled:";
        setCursor(43, 5); std::cout << "Brake Status:";
        //setCursor(43, 6); std::cout << "Matlab Sync:";
        setCursor(43, 7); std::cout << "Control Mode:";
        setCursor(43, 9); std::cout << "Timer:";

        // Draw static labels for settings panel
        setColor(GRAY);
        setCursor(3, 15); std::cout << " [E] Enable/Hold";
        setCursor(3, 16); std::cout << " [F] Disable";
        setCursor(3, 17); std::cout << " [X] E-STOP";
        setCursor(3, 18); std::cout << " [H] Home (to 0)";
        setCursor(3, 19); std::cout << " [C] Clear Faults";
        setCursor(3, 20); std::cout << " [B] Toggle Brake";
        //setCursor(3, 21); std::cout << " [L] Toggle Matlab";
        setCursor(3, 22); std::cout << " [T] Reset Timer";
    }
    
    std::deque<std::string> log_messages;
    void addLog(const std::string& msg) {
        std::lock_guard<std::mutex> lock(console_mutex);
        if (log_messages.size() >= 3) {
            log_messages.pop_front();
        }
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "[%H:%M:%S] ");
        log_messages.push_back(oss.str() + msg);

        for(int i=0; i<3; ++i) {
            setCursor(3, 27 + i);
            std::cout << std::string(74, ' ');
        }
        setColor(GRAY);
        for(size_t i=0; i<log_messages.size(); ++i) {
            setCursor(3, 27 + i);
            std::cout << log_messages[i];
        }
    }
} // namespace ui

// =================================================================================
// Utility Structures & Functions
// =================================================================================

struct PortInfo { std::string name; std::string description; };
std::vector<PortInfo> enumerateSerialPorts() {
    std::vector<PortInfo> ports;
    HDEVINFO hDevInfo = SetupDiGetClassDevsA(&GUID_DEVCLASS_PORTS, NULL, NULL, DIGCF_PRESENT);
    if (hDevInfo == INVALID_HANDLE_VALUE) return ports;
    SP_DEVINFO_DATA devInfoData;
    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
    for (DWORD i = 0; SetupDiEnumDeviceInfo(hDevInfo, i, &devInfoData); i++) {
        char friendlyName[256] = {0};
        char portName[256] = {0};
        if (SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfoData, SPDRP_FRIENDLYNAME, NULL, (PBYTE)friendlyName, sizeof(friendlyName), NULL)) {
            HKEY hDeviceKey = SetupDiOpenDevRegKey(hDevInfo, &devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
            if (hDeviceKey != INVALID_HANDLE_VALUE) {
                DWORD portNameSize = sizeof(portName);
                if (RegQueryValueExA(hDeviceKey, "PortName", NULL, NULL, (PBYTE)portName, &portNameSize) == ERROR_SUCCESS) {
                    ports.push_back({ portName, friendlyName });
                }
                RegCloseKey(hDeviceKey);
            }
        }
    }
    SetupDiDestroyDeviceInfoList(hDevInfo);
    return ports;
}

std::string selectPort(const std::string& title) {
    while (true) {
        ui::clear();
        ui::setColor(ui::WHITE);
        std::cout << "\n====================================================\n";
        std::cout << "   " << title << "\n";
        std::cout << "====================================================\n\n";
        auto ports = enumerateSerialPorts();
        if (ports.empty()) {
            std::cout << "No COM ports found! Press R to Rescan, Q to Quit.";
            int ch = _getch(); if (ch == 'q' || ch == 'Q') exit(0); continue;
        }
        int selected = 0;
        while (true) {
            ui::setCursor(0, 5);
            for (size_t i = 0; i < ports.size(); ++i) {
                if (static_cast<int>(i) == selected) { ui::setColor(ui::BLACK, ui::WHITE); std::cout << " > "; }
                else { ui::setColor(ui::WHITE, ui::BLACK); std::cout << "   "; }
                std::cout << std::left << std::setw(60) << ports[i].description + " [" + ports[i].name + "]" << "\n";
            }
            ui::setColor(ui::WHITE, ui::BLACK);
            int ch = _getch();
            if (ch == 224) { int key = _getch(); if (key == 72) selected = (selected - 1 + ports.size()) % ports.size(); else if (key == 80) selected = (selected + 1) % ports.size(); }
            else if (ch == 13) { ui::clear(); return ports[selected].name; }
            else if (ch == 'r' || ch == 'R') break;
            else if (ch == 'q' || ch == 'Q') exit(0);
        }
    }
}

std::string decodeFaults(uint8_t fault_code) {
    if (fault_code == 0) return "No Faults";
    std::string s;
    if (fault_code & 0x01) s += "V "; if (fault_code & 0x02) s += "C "; if (fault_code & 0x04) s += "T ";
    if (fault_code & 0x08) s += "E "; if (fault_code & 0x40) s += "Hw "; if (fault_code & 0x80) s += "Sw ";
    return s;
}

enum class ControlMode { ProfilePos = 0, StandardPos, Manual, COUNT };
const char* modeToString(ControlMode mode) {
    switch (mode) {
        case ControlMode::ProfilePos: return "PROFILE POS";
        case ControlMode::StandardPos: return "STD POSITION";
        case ControlMode::Manual: return "MANUAL (VEL)";
        default: return "UNKNOWN";
    }
}

struct UIElement {
    std::string name; enum Type { Value, Mode } type;
    double* value; double step;
    std::string hint;
};

// =================================================================================
// Main Application
// =================================================================================
int main(int argc, char* argv[]) {
    SetConsoleOutputCP(437);
    // --- Initialization ---
    std::string motor_port_name;
    uint8_t device_address = 1;
    if (argc >= 2) {
        motor_port_name = argv[1];
        if (argc >= 3) device_address = static_cast<uint8_t>(std::stoi(argv[2]));
    } else {
        motor_port_name = selectPort("SELECT MOTOR PORT");
        ui::setColor(ui::WHITE);
        std::cout << "\nEnter Device Address [Default=1]: ";
        std::string line;
        std::getline(std::cin, line);
        if (!line.empty()) device_address = static_cast<uint8_t>(std::stoi(line));
    }

    auto serial_port = std::make_shared<steadywin::WindowsSerialPort>();
    if (!serial_port->open(motor_port_name, 115200)) {
        ui::setColor(ui::LIGHT_RED);
        std::cerr << "Fatal Error: Failed to open port " << motor_port_name << std::endl;
        return 1;
    }
    
    steadywin::SteadywinMotor motor(device_address, serial_port);
    if(motor.initialize() != MotorError::Ok){
        ui::addLog("Motor initialization failed! Check address & connection.");
    }
    
    // --- State Variables ---
    std::atomic<bool> running{ true };
    double angle_step = 45.0;
    double current_velocity_target = 0.0;
    double pos_speed_limit = 30.0;
    double target_angle_deg = 0.0;
    VelocityControlProfile profile;
    profile.p_gain = 1.0; 
    profile.i_gain = 0.0;
    profile.d_gain = 0.0;
    profile.max_velocity_rpm = 30.0;
    profile.acceleration_rpm_s = 30.0;
    std::atomic<ControlMode> current_mode{ ControlMode::ProfilePos };
    std::atomic<bool> target_initialized{ false };
    std::atomic<bool> matlab_sync_enabled{ false };
    bool brake_closed = false;
    auto timer_start = std::chrono::steady_clock::now();
    bool timer_running = false;
    int selected_setting = 0;

    std::vector<UIElement> settings = {
        {"Control Mode",   UIElement::Mode, nullptr, 0, "Changes control algorithm (Profiled PID vs Standard)"},
        {"Target Angle",   UIElement::Value, &target_angle_deg, 1.0, "Set desired angle for position modes. A/D keys also change this."},
        {"Step Angle",     UIElement::Value, &angle_step, 1.0, "Set manual step size for A/D keys"},
        {"Pos Speed Lim",  UIElement::Value, &pos_speed_limit, 5.0, "Speed limit for internal STD POSITION mode"},
        {"P Gain",         UIElement::Value, &profile.p_gain, 0.5, "Proportional gain for PROFILE POS mode"},
        {"I Gain",         UIElement::Value, &profile.i_gain, 0.1, "Integral gain for PROFILE POS mode"},
        {"D Gain",         UIElement::Value, &profile.d_gain, 0.01, "Derivative gain for PROFILE POS mode"},
        {"Max Vel (RPM)",  UIElement::Value, &profile.max_velocity_rpm, 5.0, "Velocity limit for PROFILE POS mode"},
        {"Accel (RPM/s)",  UIElement::Value, &profile.acceleration_rpm_s, 10.0, "Acceleration limit for PROFILE POS mode"}
    };

    Telemetry initial_t;
    for(int i=0; i<5; ++i) {
        if (motor.getTelemetry(initial_t) == MotorError::Ok) {
            target_angle_deg = initial_t.multi_turn_angle_deg;
            target_initialized = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // --- Draw static UI layout once ---
    ui::drawLayout(motor_port_name, device_address);
    ui::addLog("Terminal started. Use arrow keys to navigate settings.");

    // --- Control Thread ---
    std::thread control_thread([&]() {
        double integral_error = 0.0, last_error = 0.0, current_cmd_vel = 0.0;
        auto last_time = std::chrono::steady_clock::now();
        
        /*
        // Winsock Init
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
        SOCKET udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(5005);
        server_addr.sin_addr.s_addr = INADDR_ANY;
        bind(udp_socket, (sockaddr*)&server_addr, sizeof(server_addr));
        u_long sock_mode = 1; ioctlsocket(udp_socket, FIONBIO, &sock_mode);
        sockaddr_in client_addr; int client_len = sizeof(client_addr);
        */

        while (running) {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_time).count();
            last_time = now; if (dt <= 0.001) dt = 0.001;

            Telemetry tel;
            if(motor.getTelemetry(tel) != MotorError::Ok) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Don't spam on error
                continue;
            }

            if (!target_initialized) {
                target_angle_deg = tel.multi_turn_angle_deg;
                target_initialized = true;
            }

            /*
            if (matlab_sync_enabled) {
                char buf[256];
                int n = recvfrom(udp_socket, buf, sizeof(buf)-1, 0, (sockaddr*)&client_addr, &client_len);
                if (n > 0) {
                    buf[n] = 0; try { target_angle_deg = std::stod(buf); } catch (...) {}
                    std::string resp = std::to_string(tel.multi_turn_angle_deg) + "\n";
                    sendto(udp_socket, resp.c_str(), (int)resp.size(), 0, (sockaddr*)&client_addr, sizeof(client_addr));
                }
            }
            */
            
            if (current_mode == ControlMode::ProfilePos && tel.raw_motor_status != 0) {
                double error = target_angle_deg - tel.multi_turn_angle_deg;
                integral_error += error * dt;
                double max_i = profile.max_velocity_rpm * 0.5;
                if (profile.i_gain > 0) {
                    if (integral_error * profile.i_gain > max_i) integral_error = max_i / profile.i_gain;
                    if (integral_error * profile.i_gain < -max_i) integral_error = -max_i / profile.i_gain;
                }
                double derivative = (error - last_error) / dt;
                last_error = error;

                double requested_vel = (profile.p_gain * error) + (profile.i_gain * integral_error) + (profile.d_gain * derivative);
                
                if (requested_vel > profile.max_velocity_rpm) requested_vel = profile.max_velocity_rpm;
                if (requested_vel < -profile.max_velocity_rpm) requested_vel = -profile.max_velocity_rpm;
                
                double max_dv = profile.acceleration_rpm_s * dt;
                double dv = requested_vel - current_cmd_vel;
                if (std::abs(dv) > max_dv) current_cmd_vel += (dv > 0 ? max_dv : -max_dv);
                else current_cmd_vel = requested_vel;
                
                if (std::abs(error) < 0.1 && std::abs(tel.velocity_rpm) < 1.0) {
                    current_cmd_vel = 0;
                    integral_error = 0; // Prevent windup
                }

                motor.setVelocity(current_cmd_vel);
            } else { current_cmd_vel = 0; integral_error = 0; }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        //closesocket(udp_socket); WSACleanup();
    });

    // --- Main UI Loop ---
    while (running) {
        // --- Input Handling ---
        if (_kbhit()) {
            int ch = _getch();
            if (ch == 224) { // Arrow keys
                int key = _getch();
                if (key == 72) selected_setting = (selected_setting - 1 + settings.size()) % settings.size(); // Up
                if (key == 80) selected_setting = (selected_setting + 1) % settings.size();     // Down
                if (key == 75) { // Left
                    if (settings[selected_setting].type == UIElement::Mode) current_mode = static_cast<ControlMode>((static_cast<int>(current_mode.load()) - 1 + (int)ControlMode::COUNT) % (int)ControlMode::COUNT);
                    else { *settings[selected_setting].value -= settings[selected_setting].step; if(settings[selected_setting].name == "Pos Speed Lim") motor.setPositionSpeedLimit(pos_speed_limit); }
                }
                if (key == 77) { // Right
                    if (settings[selected_setting].type == UIElement::Mode) current_mode = static_cast<ControlMode>((static_cast<int>(current_mode.load()) + 1) % (int)ControlMode::COUNT);
                    else { *settings[selected_setting].value += settings[selected_setting].step; if(settings[selected_setting].name == "Pos Speed Lim") motor.setPositionSpeedLimit(pos_speed_limit); }
                }
            }
            else if (ch == 'q' || ch == 'Q') running = false;
            else if (ch == 'e' || ch == 'E') { motor.holdPosition(); Telemetry t; if (motor.getTelemetry(t) == MotorError::Ok) target_angle_deg = t.multi_turn_angle_deg; ui::addLog("Motor ENABLED."); timer_start = std::chrono::steady_clock::now(); timer_running = true; }
            else if (ch == 'f' || ch == 'F') { motor.disable(); current_velocity_target = 0; ui::addLog("Motor DISABLED."); timer_running = false; }
            else if (ch == 'c' || ch == 'C') { motor.clearFaults(); ui::addLog("Faults CLEARED."); }
            else if (ch == 'b' || ch == 'B') { brake_closed = !brake_closed; motor.setBrake(brake_closed); ui::addLog(brake_closed ? "Brake ENGAGED." : "Brake RELEASED."); }
            //else if (ch == 'l' || ch == 'L') { matlab_sync_enabled = !matlab_sync_enabled; ui::addLog(matlab_sync_enabled ? "Matlab Sync ON." : "Matlab Sync OFF."); }
            else if (ch == 't' || ch == 'T') { timer_start = std::chrono::steady_clock::now(); ui::addLog("Timer reset."); }
            else if (ch == 'a' || ch == 'A') { target_angle_deg -= angle_step; if (current_mode == ControlMode::StandardPos) motor.moveTo(target_angle_deg); }
            else if (ch == 'd' || ch == 'D') { target_angle_deg += angle_step; if (current_mode == ControlMode::StandardPos) motor.moveTo(target_angle_deg); }
            else if (ch == 'x' || ch == 'X') { current_velocity_target = 0; motor.setVelocity(0); Telemetry t; if (motor.getTelemetry(t) == MotorError::Ok) target_angle_deg = t.multi_turn_angle_deg; motor.holdPosition(); ui::addLog("EMERGENCY STOP."); }
            else if (ch == 'h' || ch == 'H') { target_angle_deg = 0; if (current_mode == ControlMode::StandardPos) motor.moveTo(0); ui::addLog("Homing to 0 deg."); }
        }

        // --- UI Rendering ---
        std::lock_guard<std::mutex> lock(ui::console_mutex);
        Telemetry tel; motor.getTelemetry(tel);
        double elapsed = timer_running ? std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timer_start).count() / 1000.0 : 0.0;
        
        // Telemetry Panel
        ui::setColor(ui::LIGHT_CYAN);
        ui::setCursor(3, 4);  std::cout << "Angle (Multi): " << std::fixed << std::setprecision(2) << std::setw(10) << tel.multi_turn_angle_deg << " deg";
        ui::setCursor(3, 5);  std::cout << "Angle (Single):" << std::fixed << std::setprecision(2) << std::setw(10) << tel.single_turn_angle_deg << " deg";
        ui::setCursor(3, 6);  std::cout << "Velocity:      " << std::fixed << std::setprecision(2) << std::setw(10) << tel.velocity_rpm << " RPM";
        ui::setCursor(3, 7);  std::cout << "Current:       " << std::fixed << std::setprecision(3) << std::setw(10) << tel.q_axis_current_amps << " A  ";
        ui::setCursor(3, 8);  std::cout << "Voltage:       " << std::fixed << std::setprecision(2) << std::setw(10) << tel.bus_voltage_volts << " V  ";
        ui::setCursor(3, 9);  std::cout << "Temperature:   " << std::setw(10) << (int)tel.temperature_celsius << " C  ";
        ui::setCursor(3, 10); std::cout << "Faults:        "; ui::setColor(tel.raw_fault_code ? ui::LIGHT_RED : ui::LIGHT_GREEN); std::cout << std::left << std::setw(17) << decodeFaults(tel.raw_fault_code) << " ";

        // Status Panel
        ui::setCursor(58, 4); tel.raw_motor_status ? ui::setColor(ui::LIGHT_GREEN) : ui::setColor(ui::LIGHT_RED); std::cout << (tel.raw_motor_status ? "[ ENABLED  ]" : "[ DISABLED ]");
        ui::setCursor(58, 5); brake_closed ? ui::setColor(ui::LIGHT_YELLOW) : ui::setColor(ui::GRAY); std::cout << (brake_closed ? "[ CLOSED   ]" : "[ OPEN     ]");
        //ui::setCursor(58, 6); matlab_sync_enabled ? ui::setColor(ui::LIGHT_GREEN) : ui::setColor(ui::GRAY); std::cout << (matlab_sync_enabled ? "[ ACTIVE   ]" : "[ INACTIVE ]");
        ui::setColor(ui::WHITE); ui::setCursor(58, 7); std::cout << std::left << std::setw(15) << modeToString(current_mode) << " ";
        ui::setColor(ui::LIGHT_YELLOW); ui::setCursor(58, 9); std::cout << std::fixed << std::setprecision(2) << std::setw(10) << elapsed << " s   ";
        
        // Settings Panel
        for (size_t i = 0; i < settings.size(); ++i) {
            bool is_selected = ((int)i == selected_setting);
            ui::setCursor(41, 15 + i);
            if(is_selected) ui::setColor(ui::BLACK, ui::CYAN); else ui::setColor(ui::CYAN);
            std::cout << (is_selected ? " > " : "   ") << std::left << std::setw(15) << settings[i].name << ": ";
            
            if(is_selected) ui::setColor(ui::BLACK, ui::WHITE); else ui::setColor(ui::WHITE);
            std::cout << " ";
            if (settings[i].type == UIElement::Mode) std::cout << std::left << std::setw(15) << modeToString(current_mode);
            else std::cout << std::fixed << std::setprecision(2) << std::setw(10) << *settings[i].value;
            std::cout << " ";
        }

        // Hint bar for selected setting
        ui::setColor(ui::BLACK, ui::GRAY);
        ui::setCursor(0, 25);
        std::string hint_text = " HINT: " + settings[selected_setting].hint;
        std::cout << std::left << std::setw(80) << hint_text;

        ui::setColor(ui::WHITE, ui::BLACK);
        ui::setCursor(0, 32); // Hide cursor
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // --- Cleanup ---
    running = false;
    if (control_thread.joinable()) control_thread.join();
    ui::clear();
    return 0;
}
