#! /bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import re

from langchain.chat_models import ChatOpenAI
from langchain.prompts import ChatPromptTemplate
from langchain.schema.runnable import RunnablePassthrough
from langchain.document_loaders import PyPDFLoader
from langchain.text_splitter import CharacterTextSplitter
from langchain.vectorstores import FAISS
from langchain.embeddings.openai import OpenAIEmbeddings
from dotenv import load_dotenv
import gradio as gr


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        self.pattern = r"\(\s*\d+(\.\d+)?\s*,\s*\d+(\.\d+)?\s*\)"

        self.publisher_points = self.create_publisher(
            msg_type = Float32MultiArray,
            topic = '/waypoints',
            qos_profile=10)

        self.load()

        self.run()

        self.get_logger().info("LLM Node created successfully")

    def load(self):
        load_dotenv()

        model = ChatOpenAI(model="gpt-3.5-turbo")

        loader = PyPDFLoader("./chat/chat/data/points.pdf")
        pages = loader.load_and_split()

        text_splitter = CharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=0
        )

        vectorstore = FAISS.from_documents(pages, OpenAIEmbeddings())

        retriever = vectorstore.as_retriever()

        prompt = ChatPromptTemplate.from_template(
        """Answer the question based on the following context:
        {context}

        Question: {question}
        """)

        self.chain = (
            {"context": retriever, "question": RunnablePassthrough()}
            | prompt
            | model
        )

    def publish_command(self, bot_message):
        matched = re.search(self.pattern, bot_message)

        if matched:
            self.x = float(matched.group(1))
            self.y = float(matched.group(2))

            points = Float32MultiArray()
            points.data.append(self.x)
            points.data.append(self.y)

            print(points)
            
            self.publisher_points.publish(points)
            self.get_logger().info("Não encontrei os pontos")
        
        else:
            self.get_logger().info("Não encontrei os pontos")

        print(f"[RESPONSE] [CHATBOT] {bot_message}")
        
    def run(self):
        with gr.Blocks() as demo:
            chatbot = gr.Chatbot()
            msg = gr.Textbox()
            clear = gr.ClearButton([msg, chatbot])

            def respond(message, chat_history):
                bot_message = ''
                for s in self.chain.stream(message):
                    bot_message += s.content
                    self.get_logger().info(bot_message)

                self.publish_command(bot_message)

                chat_history.append((message, bot_message))
                
                return "", chat_history

            msg.submit(respond, [msg, chatbot], [msg, chatbot])

        demo.launch()

def main(args=None):
    rclpy.init(args=args)
    llm_node = LLMNode()

    rclpy.spin(llm_node)

    llm_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()