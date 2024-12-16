#pragma once

class Led {
public:
    static Led& getInstance() {
        static Led instance;
        return instance;
    }

    void turnRed();      
    void turnGreen();    
    void turnAmber();    
    void turnOff();      

private:
    Led();                
    ~Led() = default;             

    void configureIO();  

private:
    static constexpr uint8_t amberDelayMsGreen_ = 9;
    static constexpr uint8_t amberDelayMsRed_ = 6; 

    Led(const Led&) = delete;
    Led& operator=(const Led&) = delete;
};
