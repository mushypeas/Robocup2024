import openai
import json
import threading

# Read data from file
def readData(filePath):
    with open(filePath, 'r') as file:
        data = file.read()
    return data

# Load gpsr_config.json
def loadConfig(configFile):
    with open(configFile) as f:
        config = json.load(f)
    return config

# Chat with GPT-4
def chat(prompt, timeout=10):
    credentials = loadConfig('task/gpsr_repo/gpsr_credentials.json')
    openai.api_key = credentials['openai_api_key']
    modelEngine = "gpt-4"
    
    class OpenAIRequestThread(threading.Thread):
        def __init__(self, prompt):
            threading.Thread.__init__(self)
            self.prompt = prompt
            self.response = None

        def run(self):
            try:
                self.response = openai.ChatCompletion.create(
                    model=modelEngine,
                    messages=[{"role": "user", "content": self.prompt}],
                    max_tokens=1024,
                    n=1,
                    stop=None,
                    temperature=0.7,
                )
            except Exception as e:
                self.response = str(e)

    request_thread = OpenAIRequestThread(prompt)
    request_thread.start()
    request_thread.join(timeout)

    if request_thread.is_alive():
        return "The request timed out."
    else:
        message = request_thread.response.choices[0].message.content
        return message