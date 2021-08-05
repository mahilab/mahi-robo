#include <iostream>
#include <Mahi/Daq.hpp>
#include <Mahi/Gui.hpp>
#include <Mahi/Robo.hpp>
#include <thread>
#include <mutex>
#include <Mahi/Util.hpp>
#include <Mahi/Robo/Mechatronics/AtiSensor.hpp>

using namespace mahi::gui;
using namespace mahi::util;
using namespace mahi::robo;
using namespace mahi::daq;


struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};

class MyGui : public Application
{
public:
     MyGui(): Application(500, 500, "MyGui")
    {
        q8.enable();

        sensorTest.set_force_calibration(0,2,0);
        sensorTest.set_channel(&q8.AI[7]);
        q8.read_all();
        sensorTest.zero();
        control_thread = std::thread(&MyGui::control_loop, this);

    }

    ~MyGui(){
        q8.disable();
        q8.close();
        stop = true;
        control_thread.join();
    }

    void update() override
    {
        q8.read_all();
        ImGui::Begin("my widget", &open);
        
        std::cout << "sono qui 1" << std::endl;

        {   std::lock_guard<std::mutex> lock(mtx);

            if (ImGui::Button("Zero Force")){
                sensorTest.zero();
            }

            if (ImGui::Checkbox("Follow Sine", &followSine))
                toff = time();

            if (followSine) {
                inputValue= 9 * std::sin(2 * mahi::util::PI * 0.25 * time().as_seconds() - toff.as_seconds());
            }
            else {
                ImGui::DragDouble("Input Voltage", &inputValue, 0.1f, 0, 9);
            }

        }

        t += ImGui::GetIO().DeltaTime;
        
        std::cout << t << " " << senValue << std::endl;
        senData.AddPoint(t, senValue* 1.0f);
        std::cout << "sono qui 4" << std::endl;
        ivData.AddPoint(t, inputValue* 1.0f);

        ImGui::SliderFloat("History",&history,1,30,"%.1f s");

        static ImPlotAxisFlags ft_axis = ImPlotAxisFlags_NoTickLabels;
        ImPlot::SetNextPlotLimitsX(t - history, t, ImGuiCond_Always);
        if (ImPlot::BeginPlot("##Scrolling", NULL, NULL, ImVec2(-1,-1), 0, 0, 0)) {
            ImPlot::PlotLine("Output Data", &senData.Data[0].x, &senData.Data[0].y, senData.Data.size(), senData.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Input Data", &ivData.Data[0].x, &ivData.Data[0].y, ivData.Data.size(), ivData.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }

        ImGui::End();
        q8.write_all();

        if (!open){
            quit();    
        }
    }

    void control_loop() {
        Timer timer(hertz(1000));
        Time t = Time::Zero;    
        while(!stop) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                // Motor 1 - Normal Force ////////////////////////////////////
                senValue = sensorTest.get_force(Axis::AxisX);
                q8.AO[3] = inputValue;
            }

            t = timer.wait();
        }
    }

 bool open = true;

    Q8Usb q8;
    AIForceSensor sensorTest;

    double inputValue = 0;
    double senValue = 0;
    bool followSine = false;
    Time toff = Time::Zero;

    ScrollingBuffer senData, ivData;
    float t = 0;
    float history = 10.0f;

    std::thread control_thread;
    std::atomic_bool stop = false;
    std::mutex mtx;

};

int main(int, char **)
{
    MyGui gui;
    gui.run();
    return 0;
}



