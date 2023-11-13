#include "chat/chat.h"
#include <memory>

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto chat_bot = std::make_shared<ChatBot>();
   rclcpp::spin(chat_bot);
   rclcpp::shutdown();
   return 0;
}