# Gateway Reachability (ESP-NOW ACK)

## Mục tiêu
Gateway phản hồi ACK cho heartbeat của node qua ESP-NOW để node xác định trạng thái `gateway_reachable`.

## Thành phần liên quan
- Gateway:
  - `src/main.cpp`
  - `src/lib/gateway_reachability_ack.h`
  - `src/lib/gateway_reachability_ack.cpp`
- Node (tham chiếu giao thức):
  - `iot-core-communication/src/lib/GatewayReachability.h`
  - `iot-core-communication/src/lib/discovery_protocol.h`

## Message type dùng cho reachability
- `MSG_TYPE_HEARTBEAT = 2` (node -> gateway)
- `MSG_TYPE_DISCOVER = 10`
- `MSG_TYPE_DISCOVER_REPLY = 11`
- `MSG_TYPE_GATEWAY_ACK = 12` (gateway -> node)

## Cấu trúc payload reachability
Gateway sử dụng `ReachabilityMessage` (tương thích `NodeMessage` bên node):
- `device_id[32]`
- `node_id[32]`
- `node_type[16]`
- `sensor_timestamp`
- `rssi`
- `message_type`
- `uptime_sec`
- `heartbeat_seq`
- `status_kv[116]`

`static_assert(sizeof(ReachabilityMessage) <= ESP_NOW_MAX_DATA_LEN)` để đảm bảo không vượt giới hạn ESP-NOW.

## Luồng nhận/gửi ở Gateway
Trong `OnDataRecv(...)` của `src/main.cpp`:
1. Gọi `handleReachabilityMessage(mac, incomingData, len, gateway_id, WiFi.RSSI())`.
2. Nếu hàm trả về `true` thì `return` sớm (message reachability đã được xử lý).
3. Nếu trả về `false`, tiếp tục luồng xử lý dữ liệu sensor/control hiện có.

Trong `src/lib/gateway_reachability_ack.cpp`:
1. Kiểm tra `len == sizeof(ReachabilityMessage)`.
2. Parse payload thành `ReachabilityMessage`.
3. Nếu `message_type == MSG_TYPE_HEARTBEAT`:
   - Add peer ESP-NOW động theo MAC nguồn (nếu chưa có).
   - Tạo ACK với:
     - `message_type = MSG_TYPE_GATEWAY_ACK`
     - `heartbeat_seq = incoming.heartbeat_seq` (giữ nguyên seq để node match ACK)
     - `node_type = "gateway"`
     - `status_kv = "gateway_ack=true"`
   - Gửi ACK về đúng `srcMac` bằng `esp_now_send(...)`.
4. Nếu là `DISCOVER`, `DISCOVER_REPLY`, hoặc `GATEWAY_ACK` thì bỏ qua và trả `true`.

## Điều kiện node đánh dấu `gateway_reachable`
Node chỉ chấp nhận ACK khi:
- `message_type == MSG_TYPE_GATEWAY_ACK`
- `heartbeat_seq` ACK khớp probe đang chờ

Nếu timeout liên tiếp vượt ngưỡng thì node chuyển `gateway_reachable=false`; khi ACK ổn định trở lại thì phục hồi `true` theo `GatewayReachabilityConfig`.

## Log debug quan trọng
- Khi nhận heartbeat reachability:
  - `Reachability heartbeat <- <mac> | node=<node_id> | seq=<seq>`
- Khi gửi ACK thành công:
  - `Gateway ACK sent -> <mac> | seq=<seq>`
- Khi gửi ACK thất bại:
  - `Gateway ACK send failed -> <mac> | seq=<seq> | err=<code>`

## Lưu ý triển khai
- Gateway phải `ensureEspNowReady()` trước khi xử lý ESP-NOW.
- ACK chỉ phản hồi đúng MAC nguồn để tránh broadcast không cần thiết.
- Không trộn nhầm payload sensor/control với reachability: đã chặn bằng điều kiện `len == sizeof(ReachabilityMessage)`.
