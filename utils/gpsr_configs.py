########################################################
#
# GPSR MANIPULATION COMMAND TYPES
#
########################################################
command_type = []

command_type.append(['get', 'grasp', 'pick', 'up', 'take', 'the', 'from', 'to'])
# bring the chocolate drink from the kitchen(room) to the side table(placement) [0]
command_type.append(['bring', 'give', 'me', 'the', 'from'])
# bring me the banana from the side table(placement) [1]
command_type.append(['bring', 'take', 'the', 'from', 'and', 'give', 'deliver', 'it', 'to', 'at', 'the'])
# bring the banana from the side table(placement) and deliver it to (me/(James at the dining table(beacon))) [2]
command_type.append(['get', 'grasp', 'pick', 'up', 'take', 'the', 'from', 'and', 'put', 'place', 'it', 'on'])
# bring the banana from the side table and put it on the dining table [3]
command_type.append(['get', 'grasp', 'pick', 'up', 'take', 'my', 'bag', 'baggage', 'valise', 'suitcase', 'trolley',
                     'to', 'the', 'taxi', 'cap', 'uber'])
# bring my luggage to the taxi [4]
command_type.append(['bring', 'take', 'the', 'to'])  # TODO: meta
# bring the banana to the end table(placement) [5]
command_type.append(['bring', 'give', 'me', 'the', 'left', 'right', 'most', 'object', 'from'])  # TODO: meta
# bring me the right most object from the side table [6]
command_type.append(['bring', 'give', 'me', 'the', 'object', 'at', 'right', 'left', 'of', 'on', 'top', 'above', 'behind',
                     'under', 'from'])  # TODO: meta
# bring me the object under the banana from the side table [7]
command_type.append(['bring', 'give', 'me', 'the', 'biggest', 'object', 'largest', 'smallest', 'heaviest', 'lightest',
                     'thinnest', 'from'])  # TODO: meta
# bring me the biggest (object/fruit) from the side table [8]
command_type.append(['clean', 'out', 'up', 'tidy', 'neaten', 'order', 'the'])  # TODO: meta: 3 object, least 1 at floor
# clean up the living room [9]
command_type.append(['take', 'out', 'dump', 'the', 'litter', 'garbage', 'trash', 'waste', 'debris', 'junk'])
# take out the garbage [10]
command_type.append(['bring', 'give', 'deliver', 'the', 'to', 'me', 'person', 'in'])
# bring the banana to (me/the person waving in the living room) [11]
command_type.append(['go', 'navigate', 'to', 'the', 'find', 'locate', 'look', 'for', 'and', 'bring', 'give', 'deliver',
                     'it', 'at'])
# go to the end table , find the banana, and deliver it to (me/James at the kitchen) [12]
command_type.append(['go', 'navigate', 'to', 'the', 'find', 'locate', 'look', 'for', 'and', 'put', 'place', 'it', 'on'])
# go to the end table, find the banana, and put it on the side table [13]
# TODO: what if room?
# TODO: bring me the?


########################################################
#
# GPSR HRI COMMAND TYPES
#
########################################################
command_type.append(['follow', 'from', 'the', 'to'])
# follow james from the bed to the living room [14]
command_type.append(['meet', 'at', 'the', 'and', 'follow', 'him', 'her', 'to'])
# meet James at the bed and follow him to the living room [15]
command_type.append(['go', 'navigate', 'to', 'the', 'meet', 'and', 'follow', 'him', 'her'])
# go to the bed, meet James, and follow him to the living room [16]
command_type.append(['guide', 'escort', 'take', 'lead', 'accompany', 'from', 'the', 'to'])
# guide James from the bed to the desk [17]
command_type.append(['meet', 'at', 'the', 'guide', 'escort', 'take', 'lead', 'accompany', 'him', 'her', 'to'])
# meet James at the bed and guide him to the desk [18]
command_type.append(['guide', 'escort', 'take', 'lead', 'accompany', 'to', 'the', 'you', 'may', 'can', 'will', 'find',
                     'him', 'her', 'at'])
# guide James to the bed, you can find him at the desk [19]  # TODO: say guide James from the desk to the bed
command_type.append(['tell', 'me', 'the', 'name', 'gender', 'pose', 'of', 'the', 'person', 'at'])
# tell me the name of the person at the bed(beacon)/living room(room) [20]  # TODO
command_type.append(['tell', 'me', 'how', 'many', 'people', 'in ', 'the', 'are', 'men', 'women', 'boys', 'girls',
                     'male', 'female', 'sitting', 'standing', 'lying down'])
# tell me how many people in the living room are women/lying down [21]
command_type.append(['meet', 'at', 'the', 'follow', 'him', 'her', 'go', 'navigate', 'to'])
# meet James at the desk, follow him, and go to the living room [22]
command_type.append(['meet', 'at', 'the', 'follow', 'him', 'her', 'and', 'guide', 'escort', 'take', 'lead', 'accompany',
                     'back'])
# meet James at the desk, follow him, and guide him back [23]
command_type.append(['serve', 'arrange', 'deliver', 'distribute', 'give', 'provide', 'drinks', 'snacks', 'to',
                     'everyone', 'all', 'the', 'people', 'men', 'women', 'guests', 'elders', 'children', 'in'])
# serve drinks to all the women in the living room [24]
command_type.append(['contact', 'face', 'find', 'greet', 'at', 'the', 'front', 'back', 'main', 'rear', 'entrance', 'door',
                    'and', 'introduce', 'him', 'her', 'to', 'everyone', 'all', 'the', 'people', 'men', 'women', 'guests',
                    'elders', 'children', 'in'])
# face James at the main entrance and introduce him to everyone in the living room [25]
command_type.append(['contact', 'face', 'find', 'greet', 'at', 'the', 'and', 'ask', 'him', 'her', 'to', 'leave'])
# face James at the bed and ask him to leave [26]
command_type.append(['contact', 'face', 'find', 'greet', 'at', 'the', 'and', 'introduce', 'him', 'her', 'to'])
# face James at the bed and introduce him to Skyler at the table [27]
command_type.append(['contact', 'face', 'find', 'greet', 'at', 'the', 'and', 'guide', 'escort', 'take', 'lead',
                     'accompany', 'him', 'her', 'to', 'taxi', 'cab', 'uber'])
# face James at the bed and guide him to taxi [28]
command_type.append(['go', 'navigate', 'to', 'the', 'meet', 'and', 'guide', 'escort', 'take', 'lead', 'accompany',
                     'him', 'her'])
# go to the bed, meet James, and follow him to the end table [29]
command_type.append(['answer', 'question', 'tell', 'say', 'something', 'about', 'yourself', 'the', 'time', 'what', 'day', 'is',
                     'tomorrow', 'today', 'your', 'teams', 'name', 'country', 'affiliation', 'day', 'of', 'week',
                     'month', 'a', 'joke', 'to', 'person'])
# tell something about yourself to the person raising their left arm in the living room [30]
command_type.append(['find', 'locate', 'look', 'for', 'a', 'person', 'man', 'woman', 'boy', 'girl', 'male', 'female',
                     'sitting', 'standing', 'lying', 'down', 'in', 'the', 'and', 'answer', 'tell', 'say', 'something',
                     'about', 'yourself', 'time', 'what', 'day', 'is', 'tomorrow', 'today', 'your', 'teams', 'name',
                     'country', 'affiliation', 'day', 'of', 'week', 'month', 'joke'])
# find a person standing in the living room and tell something about yourself [31]
command_type.append(['go', 'navigate', 'to', 'the', 'find', 'locate', 'look', 'for', 'a', 'person', 'man', 'woman',
                     'boy', 'girl', 'male', 'female', 'sitting', 'standing', 'lying', 'down', 'and', 'tell', 'say',
                     'something', 'about', 'yourself', 'time', 'what', 'day', 'is', 'tomorrow', 'today', 'your', 'teams',
                     'name', 'country', 'affiliation', 'day', 'of', 'week', 'month', 'joke'])
# go to the living room, find a girl, and tell a joke [32]
command_type.append(['tell', 'me', 'how', 'many', 'there', 'are', 'on', 'the'])
# tell me how many bowl/snacks there are on the end table [33]
command_type.append(['find', 'locate', 'look', 'for', 'the', 'in'])
# find the pringles/fruits in the living room [34]
command_type.append(['tell', 'me', 'what', 'is', 'the', 'biggest', 'object', 'largest', 'smallest', 'heaviest', 'lightest',
                     'thinnest', 'on', 'the'])
# tell me what is the biggest object/fruits on the end table [35]
command_type.append(['tell', 'me', 'which', 'are', 'the', 'three', 'biggest', 'object', 'largest', 'smallest', 'heaviest',
                     'lightest', 'thinnest', 'objects', 'on'])
# tell me which are the three biggest objects/fruits on the end table [36]
command_type.append(['find', 'locate', 'look', 'for', 'three', 'in', 'the'])
# find three fruits in the living room [37]
# TODO: just follow (meet ~~ and follow him)
########################################################
#
# GPSR OBJECTS
#
########################################################
objects = [['cloth'], ['scrubby'], ['sponge'], ['cascade'], ['tray'], ['basket'], ['bag'], ['fork'], ['knife'],
           ['spoon'], ['chocolate'], ['coke'], ['grape'], ['orange'], ['sprite'], ['cereal'],
           ['noodles'], ['sausages'], ['apple'], ['orange'], ['paprika'], ['pringles'], ['cracker'],
           ['potato'], ['dish'], ['bowl'], ['cup'], ['drink'], ['juice'], ['chips'], ['peach'], ['water'], ['milk'],
           ['tonic'], ['bubble'], ['tea'], ['iced'], ['ice'], ['tuna'], ['can'], ['coffee'], ['jar'], ['sugar'], ['mustard'],
           ['banana'], ['strawberry'], ['pockys'], ['jello'], ['pear'], ['tomato'], ['soup'],
           ['cleanser'], ['tennis'], ['ball'], ['tennisball'], ['soccer'], ['soccerball'], ['dice'], ['baseball'],
           ['rubiks'], ['cube'], ['rubikscube'], ['plate'], ['red'], ['wine'], ['redwine'], ['cola'], ['juice'], ['pack'],
           ['juicepack'], ['tropical'], ['grounds'], ['lemon'], ['plum'], ['corn'], ['flakes'], ['cornflakes'], ['cheeze'],
           ['cheezeit'], ['base']]
# chocolate drink/grape juice/orange juice/potato chips/cascade pod
objects_sub_words = [['drink'], ['juice'], ['chips'], ['pod'], ['tea'], ['can'], ['jar'], ['red'], ['black'], ['soup'],
                     ['ball'], ['cube'], ['wine'], ['jello'], ['pack'], ['grounds'], ['flakes']]



########################################################
#
# GPSR OBJECT_TYPES
#
########################################################
object_types = [['cleaning'], ['supplies'], ['toys'], ['dishes'], ['drinks'], ['food'], ['fruits'], ['snacks'],
                ['food'], ['supply'], ['toy'], ['dish'], ['drink'], ['fruit']]
# cleaning stuff



########################################################
#
# GPSR GESTURES
#
########################################################
gestures = [['waving'], ['raising', 'their', 'left', 'arm'], ['raising', 'their', 'right', 'arm'],
            ['pointing', 'to', 'the', 'left'], ['pointing', 'to', 'the', 'right']]



########################################################
#
# GPSR ROOMS & LOCATIONS
#
########################################################
rooms = [['study'], ['bedroom'], ['living'], ['kitchen'], ['room'], ['livingroom']]
locations = [['entrance'], ['bed'], ['side'], ['desk'], ['exit'], ['sofa'], ['trashbin'], ['potted'], ['refrigerator'],
             ['end'], ['storage'], ['sink'], ['bedside'], ['dishwasher'], ['table'], ['chair'], ['chairs'], ['pantry'],
             ['shelf'], ['kitchen'], ['taxi'], ['cabinet'], ['coatrack'], ['armchair'], ['arm'], ['waste'],
             ['tv'], ['lamp'], ['bookshelf'], ['trash'], ['tvstand'], ['rack'], ['plant'], ['lamp'], ['basket'],
             ['stand'], ['bin'], ['tables']]  # TODO: armchair
rooms_sub_words = [['room']]
locations_sub_words = [['table'], ['tables'], ['rack'], ['plant'], ['basket'], ['stand'], ['bin'], ['chair']]


########################################################
#
# GPSR NAMES
#
########################################################
female_names = [['alex'], ['adel'], ['adele'], ['angel'], ['simon'], ['axel'], ['jane'], ['jules'], ['morgan'], ['paris'], ['robin'],
                ['charlie'], ['simone'], ['elizabeth'], ['francis'], ['jennifer'], ['linda'], ['mary'], ['patricia'],
                ['robin'], ['skyler']]
male_names = [['alex'], ['adel'], ['adele'], ['angel'], ['axel'], ['jules'], ['morgan'], ['charlie'], ['francis'], ['james'],
              ['john'], ['paris'], ['michael'], ['simone'], ['robin'], ['skyler'], ['william'], ['simon']]


########################################################
#
# GPSR Q & A
#
########################################################
questions = ['Do french like snails', 'Would you mind me kissing you on a train', 'Which French king ruled the least',
             "What's the busiest train station in Europe", 'Which is the highest mountain in Europe',
             'Which bread is most french a croissant or a baguette', 'Which is the most visited museum in the world',
             "What's France's cheese production",
             "Which 21 stage 23 day 2200 mile men's bike race is held each summer and ends at the Champs Elysees",
             'France shares a land border with what country that also immediately follows it on an alphabetical list of the English names of EU nations',
             'What colour features in the national flags of all the countries that border mainland France', 'What is Vincenzo Peruggia famous for',
             'Which airport is the biggest and busiest in France',
             'Lyon France is home to what border-spanning law enforcement agency',
             "What metallic element gets its name from France's old Latin name",
             'Which major public square  is located at the eastern end of the Champs Elysees',
             'Which are the five countries that are represented at every modern Olympics since its beginning',
             'What did Napoleon said in the Waterloo battle', 'In what city is the European Disney theme park located',
             'How big is a nanobot', 'Why most computerized robot voices tend to be female',
             "Who is the world's most expensive robot", 'What is the main source of inspiration in robotics',
             'Who crafted the word Robot', 'What does the word Robot mean',
             'Who formulated the principles of Cybernetics in 1948', 'Do you like super hero movies',
             'What did Nikola Tesla demonstrate in 1898', 'What was developed in 1978',
             'What is the shortest path to the Dark Side']
answers = ['The French eat around 30000 tons of snails a year.',
           'I would. French law forbids couples from kissing on train platforms.',
           'Louis XIX was the king of France for just 20 minutes, the shortest ever reign.',
           "Paris Gare du Nord is Europe's busiest railway station.",
           'The highest mountain in Europe is Mont Blank in the French Alps.',
           'The croissant was actually invented in Austria in the 13th century.',
           'The Louvre is the most visited museum in the world.',
           'France produces around one point seven million tons of cheese a year in around 1600 varieties',
           'That would be the Tour de France.', "I'm sure you're talking about Germany.",
           'Belgium, Luxemburg, Germany, Switzerland, Italy, and Spain, all have the red color in their flags.',
           'Vincenzo Peruggia is infamous for stealing the Mona Lisa in 1911.', 'The Charles de Gaulle Airport.',
           'Lyon, France is home to the Interpol.', "The gallium element got its name from France's old Latin name",
           'The Place De La Concorde', 'Australia, France, Great Britain, Greece, and Switzerland.', 'I surrender.',
           'The European Disney theme park is located in Paris.', 'A nanobot is 50 nano meter to 100 nano meter wide.',
           'One of the reasons is that females traditionally are lovely and caretaking.',
           "Honda's Asimo is the most expensive robot, costing circa two point five million USD.",
           'Nature, contributing to the field of bio-inspired robotics.',
           "The czech writer Karel Capek in his 1920's play Rossum's Universal Robots",
           'Labor or work. That would make me a servant.',
           'Norbert Wiener formulated the principles of Cybernetics in 1948.',
           "Yes, I do. Zack Snyder's are the best and my favorite character is Cyborg.",
           'In 1898, Nikola Tesla demonstrated the first radio-controlled vessel.',
           'The first object level robot programming language.',
           'My A star algorithm indicates the answer is Fear. Fear leads to anger, anger leads to hate, and hate leads to suffering.']
