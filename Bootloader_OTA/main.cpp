#include <FS.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <WebServer.h>

const char *ssid = "MANHU";
const char *password = "123456780";

WebServer server(80);
File uploadFile;
String currentFirmware = "";  // Biến để lưu tên file firmware vừa upload

#define BLOCK_SIZE 256
#define UART_TIMEOUT_MS 5000

uint32_t Bootloader_CalcChecksum(uint8_t *data, uint32_t len)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

bool waitForResponse(const char* expected, uint32_t timeout = 1000)
{
    uint32_t start = millis();
    String response = "";
    while (millis() - start < timeout) {
        if (Serial2.available()) {
            char c = Serial2.read();
            response += c;
            if (response.endsWith(expected)) return true;
        }
    }
    Serial.printf("Expected '%s' but got '%s'\n", expected, response.c_str());
    return false;
}

void handleUpload()
{
    HTTPUpload &upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        currentFirmware = "/" + String(upload.filename);  // Lưu tên file gốc (thêm '/' cho SPIFFS)
        uploadFile = SPIFFS.open(currentFirmware, FILE_WRITE);
        if (!uploadFile) {
            Serial.printf("Cant open file %s\n", currentFirmware.c_str());
            return;
        }
        Serial.println("Upload...");
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (uploadFile) uploadFile.write(upload.buf, upload.currentSize);
    } else if (upload.status == UPLOAD_FILE_END) {
        if (uploadFile) {
            uploadFile.close();
            Serial.println("Upload done.");
        }
    }
}

bool sendFirmwareToSTM32()
{
    if (currentFirmware == "") {
        Serial.println("No firmware uploaded yet.");
        return false;
    }

    File file = SPIFFS.open(currentFirmware, FILE_READ);
    if (!file) {
        Serial.printf("Cant open %s\n", currentFirmware.c_str());
        return false;
    }
    uint32_t fw_size = file.size();
    if (fw_size == 0) {
        Serial.println("File empty.");
        file.close();
        return false;
    }

    // Tính checksum toàn file
    uint8_t data[BLOCK_SIZE];
    uint32_t fw_sum = 0;
    uint32_t remaining = fw_size;
    while (remaining > 0) {
        int len = file.read(data, min((uint32_t)BLOCK_SIZE, remaining));
        fw_sum += Bootloader_CalcChecksum(data, len);
        remaining -= len;
    }
    file.seek(0);  // Reset vị trí file

    // 1. Xóa sạch bộ đệm Serial nhận về để tránh rác cũ
    while(Serial2.available()) {
        Serial2.read(); 
    }

    Serial.println("Waiting for BL_READY...");
    
    // 2. Chờ đúng tín hiệu BL_READY
    // Tăng timeout lên một chút để an toàn
    if (!waitForResponse("BL_READY\r\n", 6000)) { 
        Serial.println("No BL_READY received.");
        file.close();
        return false;
    }
    
    // Delay cực ngắn để đảm bảo STM32 đã chuyển sang trạng thái Receive (sau khi gửi BL_READY)
    delay(5); 

    // 3. Chuẩn bị Header
    uint8_t header[8];
    header[0] = fw_size & 0xFF;
    header[1] = (fw_size >> 8) & 0xFF;
    header[2] = (fw_size >> 16) & 0xFF;
    header[3] = (fw_size >> 24) & 0xFF;
    header[4] = fw_sum & 0xFF;
    header[5] = (fw_sum >> 8) & 0xFF;
    header[6] = (fw_sum >> 16) & 0xFF;
    header[7] = (fw_sum >> 24) & 0xFF;

    // 4. Gửi Header ngay lập tức
    Serial2.write(header, 8);
    Serial.println("Sent header.");

    // Chờ READY từ STM32
    if (!waitForResponse("READY\r\n", 2000)) {
        Serial.println("No READY received.");
        file.close();
        return false;
    }

    // Gửi data chunks
    remaining = fw_size;
    int blockCount = 0;
    while (remaining > 0) {
        int len = file.read(data, min((uint32_t)BLOCK_SIZE, remaining));
        int retry = 3;
        bool sent = false;
        while (retry-- > 0) {
            Serial2.write(data, len);
            // Không có ACK per block, nhưng chờ một chút và check error nếu có
            delay(10);  // Delay nhỏ để STM32 xử lý
            if (Serial2.available()) {
                String resp = "";
                while (Serial2.available()) resp += (char)Serial2.read();
                if (resp.indexOf("ERR") >= 0 || resp.indexOf("ABORT") >= 0) {
                    Serial.printf("Error from STM: %s\n", resp.c_str());
                    file.close();
                    return false;
                }
            }
            sent = true;  // Giả sử OK nếu không error
            break;
        }
        if (!sent) {
            Serial.println("Failed to send block.");
            file.close();
            return false;
        }
        remaining -= len;
        blockCount++;
        Serial.printf("Sent block %d\n", blockCount);
    }

    file.close();
    Serial.println("Finished sending firmware to STM32.");
    return true;
}

void setup()
{
    Serial.begin(9600);
    Serial2.begin(115200);

    SPIFFS.begin(true);

    WiFi.begin(ssid, password);
    Serial.print("WiFi connecting...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected. IP:");
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>Upload Firmware</title>
      <style>
        body { background-color: #f0f0f0; font-family: Arial, sans-serif; text-align: center; padding-top: 50px; }
        form { background: #fff; padding: 20px; border-radius: 10px; display: inline-block; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
        input[type="file"] { margin-bottom: 15px; }
        input[type="submit"], a.button { background-color: #4CAF50; color: white; padding: 10px 20px; text-decoration: none; border: none; border-radius: 5px; cursor: pointer; }
        a.button { display: inline-block; margin-top: 15px; }
      </style>
    </head>
    <body>
      <form method="POST" action="/upload" enctype="multipart/form-data">
        <h2>OTA - STM32 - ESP32</h2>
        <input type="file" name="file" accept=".bin"><br>
        <input type="submit" value="Upload BIN">
        <br>
        <a href="/send" class="button">Send to STM32</a>
      </form>
    </body>
    </html>
  )rawliteral");
    });

    server.on("/upload", HTTP_POST, []() {
        server.send(200, "text/html", R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>Upload Firmware</title>
      <style>
        body { background-color: #f0f0f0; font-family: Arial, sans-serif; text-align: center; padding-top: 50px; }
        form { background: #fff; padding: 20px; border-radius: 10px; display: inline-block; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
        input[type="file"] { margin-bottom: 15px; }
        input[type="submit"], a.button { background-color: #4CAF50; color: white; padding: 10px 20px; text-decoration: none; border: none; border-radius: 5px; cursor: pointer; }
        a.button { display: inline-block; margin-top: 15px; }
      </style>
    </head>
    <body>
      <form method="POST" action="/upload" enctype="multipart/form-data">
        <h2>OTA - STM32 - ESP32</h2>
        <a href="/send" class="button">Send to STM32</a>
      </form>
    </body>
    </html>
  )rawliteral");
    }, handleUpload);

    server.on("/send", HTTP_GET, []() {
        bool success = sendFirmwareToSTM32();
        String message = success ? "Firmware sent successfully!" : "Error sending firmware.";
        server.send(200, "text/html", R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>Sending Firmware</title>
      <style>
        body { background-color: #f0f0f0; font-family: Arial, sans-serif; text-align: center; padding-top: 50px; }
        .container { background: #fff; padding: 30px; border-radius: 15px; display: inline-block; box-shadow: 0 0 15px rgba(0,0,0,0.2); }
        h2 { color: #333; }
        p { font-size: 18px; margin: 20px 0; }
        a.button { background-color: #4CAF50; color: white; padding: 12px 24px; text-decoration: none; border-radius: 8px; cursor: pointer; font-size: 16px; }
        a.button:hover { background-color: #45a049; }
      </style>
    </head>
    <body>
      <div class="container">
        <h2>Sending Firmware</h2>
        <p>)rawliteral" + message + R"rawliteral(</p>
        <a href="/" class="button">Back to Upload</a>
      </div>
    </body>
    </html>
  )rawliteral");
    });

    server.begin();
}

void loop()
{
    server.handleClient();
}