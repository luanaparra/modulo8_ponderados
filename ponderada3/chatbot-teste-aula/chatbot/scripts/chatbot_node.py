import re

intents = {
    "go_to": {
        "patterns": [r"va para (.+)", r"dirija-se ao (.+)", r"me leve para (.+)"],
        "action": "move_robot"
    }
}

actions = {
    "move_robot": lambda location: print(f"indo para {location}")
}

def process_command(command):
    for intent, data in intents.items():
        for pattern in data['patterns']:
            match = re.match(pattern, command)
            if match:
                action_func = actions[data['action']]
                location = match.group(1)
                action_func(location)
                return True
    return False

if __name__ == "__main__":
    while True:
        user_input = input("Digite um comando: ")
        if user_input.lower() == 'sair':
            break
        if not process_command(user_input):
            print("OPS, comando não reconhecido.")
