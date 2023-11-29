import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from langchain.chat_models import ChatOpenAI
from langchain.prompts import ChatPromptTemplate
from langchain.schema.runnable import RunnablePassthrough
from langchain.document_loaders import PyPDFLoader
from langchain.text_splitter import CharacterTextSplitter
from langchain.vectorstores import FAISS
from langchain.embeddings.openai import OpenAIEmbeddings
from dotenv import load_dotenv

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        self.subscription_ = self.create_subscription(
            msg_type=String,
            topic="/llm",
            callback=self.listener_callback,
            qos_profile=10
        )
        self.publisher_ = self.create_publisher(
            msg_type=String,
            topic="/chatbot",
            qos_profile=10
        )

        try:
            self.load()
            self.get_logger().info("LLM Node created successfully")
        except Exception as e:
            self.get_logger().error(f"Error during initialization: {e}")
            self.destroy_node()
            raise

    def load(self):
        load_dotenv()

        loader = PyPDFLoader("./chat/chat/data/points.pdf")
        pages = loader.load_and_split()

        text_splitter = CharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=0
        )

        vectorstore = FAISS.from_documents(pages, OpenAIEmbeddings())

        retriever = vectorstore.as_retriever()

        template = """Answer the question based only on the following context:
        {context}

        Question: {question}
        """

        prompt = ChatPromptTemplate.from_template(template)

        model = ChatOpenAI(model="gpt-3.5-turbo")

        self.chain = (
            {"context": retriever, "question": RunnablePassthrough()}
            | prompt
            | model
        )

    def listener_callback(self, msg):
        try:
            self.get_logger().info(f"Received '{msg.data}'")
            content = ""
            for s in self.chain.stream(msg.data):
                content += s.content
            self.publish_(content)
            self.get_logger().info(f"Response published")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def publish_(self, content):
        msg = String()
        msg.data = content
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published '{content}'")

def main(args=None):
    rclpy.init(args=args)
    try:
        llm_node = LLMNode()
        rclpy.spin(llm_node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'llm_node' in locals():
            llm_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
