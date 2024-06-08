import re
import warnings
import json 

from gpsr_dicts import *
from gpsr_utils import chat
from gpsr_config import *

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

import re
import warnings

def parseObjects(data):
    parsedObjects = re.findall(r'\|\s*(\w+)\s*\|', data, re.DOTALL)
    parsedObjects = [objectName for objectName in parsedObjects if objectName != 'Objectname']
    parsedObjects = [objectName.replace("_", " ") for objectName in parsedObjects]
    parsedObjects = [objectName.strip() for objectName in parsedObjects]

    parsedCategories = re.findall(r'# Class \s*([\w,\s, \(,\)]+)\s*', data, re.DOTALL)
    parsedCategories = [category.strip() for category in parsedCategories]
    parsedCategories = [category.replace('(', '').replace(')', '').split() for category in parsedCategories]
    parsedCategoriesPlural = [category[0] for category in parsedCategories]
    parsedCategoriesPlural = [category.replace("_", " ") for category in parsedCategoriesPlural]
    parsedCategoriesSingular = [category[1] for category in parsedCategories]
    parsedCategoriesSingular = [category.replace("_", " ") for category in parsedCategoriesSingular]

    if parsedObjects or parsedCategories:
        return parsedObjects, parsedCategoriesPlural, parsedCategoriesSingular
    else:
        warnings.warn("List of objects or object categories is empty. Check content of object markdown file")
        return []

def extractCategoryToObj(markdownContent):
    categoryPattern = re.compile(r'\# Class (\w+) \((\w+)\)')
    objectPattern = re.compile(r'\| (\w+)  \|')

    objectsDict = {}
    categorySingToPlur = {}
    categoryPlurToSing = {}
    currentCategoryPlur = None
    currentCategory = None

    for line in markdownContent.split('\n'):
        categoryMatch = categoryPattern.match(line)
        if categoryMatch:
            currentCategoryPlur = categoryMatch.group(1)
            currentCategory = categoryMatch.group(2)

            categorySingToPlur[currentCategory] = currentCategoryPlur
            categoryPlurToSing[currentCategoryPlur] = currentCategory

            objectsDict[currentCategory] = []

        objectMatch = objectPattern.match(line)
        if objectMatch and currentCategory:
            objectName = objectMatch.group(1)
            objectsDict[currentCategory].append(objectName)

    return objectsDict, categoryPlurToSing, categorySingToPlur

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
    prompt += "You should know that the inputText is a command which is spoken. Always think about pronounciation similarity. \n"
    prompt += f'Infer verbType of inputText with this dict first. {{verbType: [verb]}}: {verbType2verb}\n'
    prompt += f'Then, infer cmdName of the inputText, and every {{parameters}} surrounded by braces {{cmdName: sentence {{parameter}}}}: {cmdName2cmdStr}\n'
    prompt += f'you can refer to gesture_person_list => {gesture_person_list}\n, pose_person_list => {pose_person_list}\n, gesture_person_plural_list => {gesture_person_plural_list}\n, pose_person_plural_list => {pose_person_plural_list}\n, person_info_list => {person_info_list}\n, object_comp_list => {object_comp_list}\n, talk_list => {talk_list}\n, question_list => {question_list}\n, color_list => {color_list}\n, clothe_list => {clothe_list}\n, clothes_list => {clothes_list}\n'
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