import xml.etree.ElementTree as ET

subject = 'q'

if subject == 'l':
    tree = ET.parse('Locations.xml')
    root = tree.getroot()
    rooms = []
    locations = []
    for room in range(len(root)):
        rooms.append([root[room].attrib['name']])
        for location in range(len(root[room])):
            locations.append([root[room][location].attrib['name']])
    print(rooms)
    print(locations)

elif subject == 'n':
    tree = ET.parse('Names.xml')
    root = tree.getroot()
    females = []
    males = []
    for p in range(len(root)):
        if root[p].attrib['gender'] == 'Female':
            females.append([root[p].text])
        else:
            males.append([root[p].text])
    print(females)
    print(males)

elif subject == 'o':
    tree = ET.parse('Objects.xml')
    root = tree.getroot()
    categories = []
    objects = []
    for cat in range(len(root)):
        categories.append([root[cat].attrib['name']])
        for o in range(len(root[cat])):
            objects.append([root[cat][o].attrib['name']])
    print(categories)
    print(objects)

elif subject == 'q':
    tree = ET.parse('gpsr_xmls/Questions.xml')
    root = tree.getroot()
    questions = []
    answers = []
    for q in range(len(root)):
        questions.append(root[q][0].text)
        answers.append(root[q][1].text)
    print(questions)
    print(answers)