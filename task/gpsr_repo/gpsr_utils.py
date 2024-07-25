import openai
import json

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
def chat(prompt):
    gpsrConfig = loadConfig('task/gpsr_repo/gpsr_config.json')
    openai.api_key = gpsrConfig['openai_api_key']
    modelEngine = "gpt-4"

    response = openai.ChatCompletion.create(
        model=modelEngine,
        messages=[{"role": "user", "content": prompt}],
        max_tokens=1024,
        n=1,
        stop=None,
        temperature=0.7,
    )

    return response.choices[0].message.content
