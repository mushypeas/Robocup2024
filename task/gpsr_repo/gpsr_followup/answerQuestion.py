# "answerQuestion": "{answerVerb} a {question}"
def answerQuestion(g, params):
    print("followup: answerQuestion")
    
    # [1] Get the question
    g.quiz()