import numpy as np
import time
import openai


openai.api_key = "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"
COMPLETIONS_MODEL = "text-davinci-002"

def run_gpt3(rule, question, debug=True):
    prompt = """Based on the description, \"{}\", {}, answer with yes or no?\n""".format(rule, question)
    ##prompt = """Based on the description "{}", answering the following question using yes or no: {}\n""".format(rule, question)
    ##prompt = """Based on the description "{}", {} Answer \"Yes\" or \"No\".\n""".format(rule, question)
    # prompt="in the sentence \"{}\", {} Answer \"Yes\" or \"No\".\n".format(rule, question)
    ##prompt = question
    time.sleep(2)

    result = openai.Completion.create(
        prompt=prompt,
        temperature=0,
        max_tokens=300,
        top_p=1,
        logprobs=5,
        frequency_penalty=0,
        presence_penalty=0,
        model=COMPLETIONS_MODEL
        )
    if debug:
        print(question)
        print(result["choices"][0]["text"].strip(" \n"))  
    ans = result["choices"][0]["text"].strip(" \n")[:3]
    prediction_results = result["choices"][0]["text"].replace('\n', ' ').split(' ')
    for i in range(len(prediction_results)):
        if 'yes' in prediction_results[i].lower() or 'no' in prediction_results[i].lower():
            break

    probs = np.array(list(result["choices"][0]["logprobs"]["top_logprobs"][i].values()))
#     probs = np.exp(np.array(result["choices"][0]["logprobs"]["top_logprobs"][0].values()))
    probs = np.exp(probs) / np.exp(probs).sum()
    answers = list(result["choices"][0]["logprobs"]["top_logprobs"][i].keys())
    
    prob_yes = 0
    prob_no = 0
    for ans, prob in zip(answers, probs):
        if 'yes' in ans.lower():
            prob_yes += prob
        elif 'no' in ans.lower():
            prob_no += prob

    if prob_yes >= prob_no:
        return "yes", prob_yes
    else:
        return "no", prob_no
