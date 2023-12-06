import numpy as np

class Perceptron:
    def __init__(self, input_size):
        self.weights = np.random.rand(input_size)
        self.bias = np.random.rand()
        self.learning_rate = 0.1

    def activation_function(self, x):
        return 1 if x >= 0 else 0  # Função de ativação degrau

    def predict(self, inputs):
        weighted_sum = np.dot(inputs, self.weights) + self.bias
        return self.activation_function(weighted_sum)

    def train(self, training_inputs, labels, epochs):
        for epoch in range(epochs):
            for inputs, label in zip(training_inputs, labels):
                prediction = self.predict(inputs)
                error = label - prediction
                self.weights += self.learning_rate * error * inputs
                self.bias += self.learning_rate * error

if __name__ == "__main__":
    # AND
    and_inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
    and_labels = np.array([0, 0, 0, 1])

    and_perceptron = Perceptron(2)
    and_perceptron.train(and_inputs, and_labels, epochs=100)

    print("Teste AND:")
    test_inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
    for inputs in test_inputs:
        prediction = and_perceptron.predict(inputs)
        print(f"Entrada: {inputs} -> Saída: {prediction}")

    # OR
    or_inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
    or_labels = np.array([0, 1, 1, 1])

    or_perceptron = Perceptron(2)
    or_perceptron.train(or_inputs, or_labels, epochs=100)

    print("\nTeste OR:")
    test_inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
    for inputs in test_inputs:
        prediction = or_perceptron.predict(inputs)
        print(f"Entrada: {inputs} -> Saída: {prediction}")

    # NAND
    nand_inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
    nand_labels = np.array([1, 1, 1, 0])

    nand_perceptron = Perceptron(2)
    nand_perceptron.train(nand_inputs, nand_labels, epochs=100)

    print("\nTeste NAND:")
    test_inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
    for inputs in test_inputs:
        prediction = nand_perceptron.predict(inputs)
        print(f"Entrada: {inputs} -> Saída: {prediction}")

    #XOR (não da certo)
    xor_inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
    xor_labels = np.array([0, 1, 1, 0])
    
    xor_perceptron = Perceptron(2)
    xor_perceptron.train(xor_inputs, xor_labels, epochs=100)

    print("\nTeste XOR:")
    test_inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
    for inputs in test_inputs:
        prediction = xor_perceptron.predict(inputs)
        print(f"Entrada: {inputs} -> Saída: {prediction}")


