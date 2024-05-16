import re
import warnings
import json 

from gpsr_dicts import *
from gpsr_utils import chat

class NoAppropriateVerbError(Exception):
   """
   This exception is raised when no appropriate verb is found for a given command.
   
   Attributes:
       command (str): The command for which no appropriate verb was found.
       message (str): An explanatory message about the exception.
   """

   def __init__(self, command, message="No appropriate verb found for the given command."):
       self.command = command
       self.message = message
       super().__init__(self.message)

   def __str__(self):
       return f"{self.message} Command: {self.command}"

# Parse object.md
def parse_objects(data):
    parsed_objects = re.findall(r'\|\s*(\w+)\s*\|', data, re.DOTALL)
    parsed_objects = [objects for objects in parsed_objects if objects != 'Objectname']
    parsed_objects = [objects.replace("_", " ") for objects in parsed_objects]
    parsed_objects = [objects.strip() for objects in parsed_objects]

    parsed_categories = re.findall(r'# Class \s*([\w,\s, \(,\)]+)\s*', data, re.DOTALL)
    parsed_categories = [category.strip() for category in parsed_categories]
    parsed_categories = [category.replace('(', '').replace(')', '').split() for category in parsed_categories]
    parsed_categories_plural = [category[0] for category in parsed_categories]
    parsed_categories_plural = [category.replace("_", " ") for category in parsed_categories_plural]
    parsed_categories_singular = [category[1] for category in parsed_categories]
    parsed_categories_singular = [category.replace("_", " ") for category in parsed_categories_singular]

    if parsed_objects or parsed_categories:
        return parsed_objects, parsed_categories_plural, parsed_categories_singular
    else:
        warnings.warn("List of objects or object categories is empty. Check content of object markdown file")
        return []

# Make Category to Object Dictionary
def extractCategory2obj(markdown_content):
    category_pattern = re.compile(r'\# Class (\w+) \((\w+)\)')
    object_pattern = re.compile(r'\| (\w+)  \|')

    objects_dict = {}
    categorySing2Plur = {}
    categoryPlur2Sing = {}
    current_category_plur = None
    current_category = None

    for line in markdown_content.split('\n'):
        category_match = category_pattern.match(line)
        if category_match:
            current_category_plur = category_match.group(1)
            current_category = category_match.group(2)

            categorySing2Plur[current_category] = current_category_plur
            categoryPlur2Sing[current_category_plur] = current_category

            objects_dict[current_category] = []

        object_match = object_pattern.match(line)
        if object_match and current_category:
            object_name = object_match.group(1)
            objects_dict[current_category].append(object_name)

    return objects_dict, categoryPlur2Sing, categorySing2Plur

# ULTIMATE Text Parser
def ultimateParser(inputText):
    '''Ultimate parser for the inputText. It uses GPT-4 to parse the inputText.'''
    # splitedInputText = inputText.split()
    # mainVerb = splitedInputText[0]

    # for verbType in verbType2verb:
    #     if mainVerb.lower() in verbType2verb[verbType]:
    #         print([cmdEntries for cmdEntries in cmdName2cmdStr.items()])
    #         candidateCmdStr = dict([cmdEntries for cmdEntries in cmdName2cmdStr.items() if cmdEntries[1].split()[0] == verbType])

    prompt = f'inputText: {inputText}\n'
    prompt += f'Infer verbType of inputText with this dict first. {{verbType: [verb]}}: {verbType2verb}\n'
    prompt += f'Then, infer cmdName of the inputText, and every {{parameters}} surrounded by braces {{cmdName: sentence {{parameter}}}}: {cmdName2cmdStr}\n'
    prompt += 'finally, answer which cmdName inputText is, and every {parameters} in the inputText without missing\n'
    prompt += 'you should only write with format: cmdName, {"parameterName": "parameterValue", ...}'
    
    gptAnswer = chat(prompt)

    splitIndex = gptAnswer.find(', ')
    cmdName = gptAnswer[:splitIndex]
    params = json.loads(gptAnswer[splitIndex+2:])

    ### TODO ###
    ### Catch Error and Retry

    print("[Parser] cmdName:", cmdName)
    print("[Parser] params:", params)

    return cmdName, params
    
def nogadaParser(inputText):
    '''Handcrafted parser for the inputText'''
    ### TODO ###
    ### Make Handcrafted Parser
    pass

def ultimateFollowupParser(inputText):
    '''Ultimate parser for the inputText. It uses GPT-4 to parse the inputText.'''

    prompt = f'inputText: {inputText}\n'
    prompt += f'Infer verbType of inputText with this dict first. {{verbType: [verb]}}: {verbType2verb}\n'
    prompt += f'Then, infer followupName of the inputText, and every {{parameters}} surrounded by braces {{followupName: sentence {{parameter}}}}: {followupName2followupStr}\n'
    prompt += 'finally, answer which followupName inputText is, and every {parameters} in the inputText without missing\n'
    prompt += 'you should only write with format: followupName, {"parameterName": "parameterValue", ...}'
    
    gptAnswer = chat(prompt)

    splitIndex = gptAnswer.find(', ')
    cmdName = gptAnswer[:splitIndex]
    params = json.loads(gptAnswer[splitIndex+2:])

    ### TODO ###
    ### Catch Error and Retry

    print("[Parser] cmdName:", cmdName)
    print("[Parser] params:", params)

    return cmdName, params

# TEST CODE
if __name__ == "__main__":
    followup = "and bring it to me"
    print(ultimateFollowupParser(followup))