/********
 * 
 * Treiber fuer den Historischen Fahrsimulator der Berliner S-Bahn
 * Entwickelt von: Christian Schulz & ChatGPT-4/5
 * 
 * Buildhinweise:
 * Im Ordner build mit folgendem Befehl compilieren:
 * rm -rf * && cmake .. && cmake --build . -j1 2>&1 | tee build.log
 * WICHTIG!!! Du musst dich wirklich in diesem Ordner befinden, 
 * sonst wird der ganze Bums geloescht!
 * 
 * Ausfuehren:
 * im Ordner build/bin: ./locsim_app
 * WICHTIG!!! in diesem Ordner build/bin/config muss sich die io_map.json befinden!
 * 
 * 
 * 
 * 
 */


#include <iostream>
#include <atomic>
#include <csignal>
#include <deque>
#include <mutex>
#include <condition_variable>

#include "../core/Telegram.h"
#include "../config/signals.h"
#include "../i2c/core/I2C_Manager.h"
#include "../core/IO_Broker.h"
#include "../i2c/Hardware/PCF8574In.h"
#include "../i2c/Hardware/AnalogIn.h"
#include "../i2c/Hardware/PCF8574Out.h"
#include "../i2c/Hardware/AnalogOut.h"
#include "../i2c/Hardware/MPU9250.h"

#define Diag

using namespace i2c;

int main() {

    std::deque<Telegram> TelegramVonPult;   // Briefkasten
    std::mutex           queueMutex;        // Briefkastenschluessel
    std::condition_variable Klingel;        // Klingelt bei neuer Post
    std::atomic<bool> running{true};
    /**********
     * INIT
     */
    //std::signal(SIGINT, [](int){
    //    running = false;
        // weckt den Broker-Worker (falls gerade blockiert)
        // Optional, falls dein IoBroker::stop() nicht selbst weckt:
        // i2c.enqueue(DeviceEvent{});
    //});
    // 1) Grundobjekte
    i2c::I2C_Manager i2c(1); // Manager erstellen
    i2c.setGlobalInterrupt(17, i2c::I2C_Manager::Edge::Rising, 2);
    
    
    i2c::PCF8574In pcf_in(i2c, 0x38);
    //pcf.setOnChange([](uint8_t v){ std::cout << "[0x38] Port=" << int(v) << "\n";  });
    i2c.subscribeInterrupt([&pcf_in]{ pcf_in.onInterrupt(); });

    i2c::AnalogIn adc(i2c, 0x69, {8,9,10,11}, 50);
    i2c::MPU9250 imu(i2c, 0x68, 50);
    imu.begin();

    i2c::PCF8574Out pcf_out(i2c, 0x20);
    pcf_out.begin(false);

    i2c::AnalogOut dac(i2c, 0x58);
    dac.begin();
    
    IoBroker io(i2c);   // Broker erstellen
    if(!io.loadConfig("config/io_map.json")){
        std::cerr << io.lastError() << "\n";
        return 1;
    }
    io.setOnTelegram([&](const Telegram& t){
        #ifdef Diag
        std::cout   << "IN: kanal=" << t.kanal
                        << " name=" << io.nameOf(t.kanal)
                        << " wert=" << t.wert 
                        << " (andi=" << t.analog << ")\n";
        #endif
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            TelegramVonPult.push_back(t);   // Neue Post in den queue laden
        }
        Klingel.notify_one();   // mainloop wecken: DIE POST IST DA!
    });

    io.start();   // ab jetzt kommen Events „asynchron“ rein
    /**********
     * INIT ENDE
     */

    // dein restliches Programm hier:
    // - UART-Loop
    // - TCP-Server
    // - Zustandsautomaten
    // ...

    // Mache was anderes, Mainthread!

    auto handle = [&](const Telegram& t){
        // Hier kommt die Logic-Engine rein.
        
        switch(t.kanal){
            case 0: io.publishOut(Telegram{12, false, t.wert}); break;
            case 1: io.publishOut(Telegram{13, false, t.wert}); break;
            case 2: io.publishOut(Telegram{14, false, t.wert}); break;
            case 3: io.publishOut(Telegram{15, false, t.wert}); break;
            case 4: io.publishOut(Telegram{16, false, t.wert}); break;
            case 5: io.publishOut(Telegram{17, false, t.wert}); break;
            case 6: io.publishOut(Telegram{18, false, t.wert}); break;
            case 7: io.publishOut(Telegram{19, false, t.wert}); break;

            case HBL_DRUCK: io.publishOut(Telegram{A_TACHO, true, t.wert/2000}); break;
            case HANDBREMSE: io.publishOut(Telegram{A_FAHRSPANNUNG, true, t.wert/2000}); break;
            case HL_DRUCK: io.publishOut(Telegram{A_110V, true, t.wert/2000}); break;
            case C_DRUCK: io.publishOut(Telegram{A_230V, true, t.wert/2000}); break;
        }
    };

    while (running) {
        Telegram t;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            // Warte bis Post da ist oder ein Timeout (hier 50 ms) vorbeigeht
            Klingel.wait_for(lock, std::chrono::milliseconds(50),
                            [&]{ return !TelegramVonPult.empty() || !running; });
            if (!running) break;
            if (TelegramVonPult.empty()) continue;        // Timeout: Platz für andere Jobs
            t = TelegramVonPult.front();
            TelegramVonPult.pop_front();
        }
        
        handle(t); // außerhalb des Locks: Verarbeitung darf „dauern“
    }
    
    
    running = false;
    Klingel.notify_all(); // wecke evtl. wartende Threads, damit sie sauber rausgehen

    // IO Broker Thread Stoppen
    io.stop();
    return 0;
}
