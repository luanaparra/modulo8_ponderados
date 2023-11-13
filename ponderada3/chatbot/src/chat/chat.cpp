#include "chat.h"
#include "memory"

ChatBot::ChatBot() : rclcpp::Node("chat_bot")
{
   std::regex grupo1("\\b(preciso ir para|me leve para|vai para|me leve at[ée]|vamos para|vai at[eé])\\s+(.+)\\b", std::regex_constants::icase);
   std::regex grupo2("\\b(como voc[êeéèê] pode me ajudar|o que voc[êeéèê] pode fazer|me ajud[ea])\\b", std::regex_constants::icase);
   std::regex grupo3("\\b(voc[êeéèê] chegou|voc[êeéèê] est[áaáàâã] ocupado|o que voc[êeéèê] est[áaáàâã] fazendo)\\b", std::regex_constants::icase);

   // Inicia um array de regexs
   this->regexs_ = std::vector<std::regex>({grupo1, grupo2, grupo3});

   // Inicia mapping de ações para cada comando
   actions_[0] = [](std::smatch &match)
   {
      std::cout << match[2].str() << " é logo ali me siga que te levo lá! \n"
                << std::endl;
   };

   actions_[1] = [](std::smatch &_)
   {
      std::cout << "\033[32m Você acionou AJUDA. Cá está o que posso fazer:\033[0m" << std::endl;

      std::cout << "\n digite: \033[34m me leve para <local> ou vamos para <local>\033[0m" << std::endl;
      std::cout << "\033[34m --> Vou lhe encaminhar para o local desejado \033[0m" << std::endl;

      std::cout << "\n\n\n digite: \033[34m me ajude \033[0m" << std::endl;
      std::cout << "\033[34m --> Irei te mostrar como posso te ajudar \033[0m" << std::endl;

      std::cout << "\n\n\n digite: \033[34m o que voce esta fazendo? \033[0m" << std::endl;
      std::cout << "\033[34m --> Irei lhe falar o que estou fazendo \033[0m \n\n"
                << std::endl;
   };

   actions_[2] = [](std::smatch &_)
   {
      std::cout << "Estou esperando pelo seu comando!" << std::endl;
   };

   std::cout << "Olá meu nome é Nicola, em que posso te ajudar hoje?" << std::endl;

   this->ask_command();
}

void ChatBot::ask_command()
{
   while (rclcpp::ok())
   {
      std::string command;

      std::cout << "Digite seu comando: ";

      std::getline(std::cin, command);

      if (command == "")
      {
         continue;
      }
      this->interpret_command(command);

      if (!rclcpp::ok())
         break;
   }
};

void ChatBot::interpret_command(const std::string &command)
{
   std::smatch match;
   if (std::regex_search(command, match, this->regexs_[0]) && match.size() > 1)
   {
      actions_[0](match);
   }
   else if (std::regex_search(command, match, this->regexs_[1]))
   {
      actions_[1](match);
   }
   else if (std::regex_search(command, match, this->regexs_[2]))
   {
      actions_[2](match);
   }
   else
   {
      std::cout << "Não entendi o que você falou, consegue repetir?" << std::endl;
   }
};