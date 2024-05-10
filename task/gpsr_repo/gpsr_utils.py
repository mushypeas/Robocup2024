import openai
import json

# LOAD gpsr_config.json
def load_config(config_file):
    with open(config_file) as f:
        config = json.load(f)
    return config

# CHAT w/ gpt-4
def chat(prompt):
    gpsr_config = load_config('gpsr_config.json')
    openai.api_key = gpsr_config['openai_api_key']
    model_engine = "gpt-4"

    response = openai.ChatCompletion.create(
        model=model_engine,
        messages=[{"role": "user", "content": prompt}],
        max_tokens=1024,
        n=1,
        stop=None,
        temperature=0.7,
    )

    return response.choices[0].message.content