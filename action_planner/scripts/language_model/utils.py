import openai
import torch
from sentence_transformers import util as st_utils
import numpy as np
from language_model.config import Hyperparameters
from language_model.dataset import *


if torch.cuda.is_available():
  torch.cuda.set_device(0)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def language_model_engine(planning_lm_id, OPENAI_KEY):
  client = openai.OpenAI(api_key=OPENAI_KEY)
  def _generate(prompt, sampling_params, max_tokens, stop):
    completion = client.chat.completions.create(
    model=planning_lm_id,
    messages=[
      {"role": "system", "content": prompt["system"]},
      {"role": "user", "content": prompt["user"]}
    ],
    **sampling_params)

    generated_samples = [completion.choices[i].message.content.strip().lower() for i in range(sampling_params['n'])]
    # calculate mean log prob across tokens
    mean_log_probs = [np.mean([lp.logprob for lp in completion.choices[i].logprobs.content]) for i in range(sampling_params['n'])]
    return generated_samples, mean_log_probs

  return _generate


# hyperparameters for plangeneration
def get_hyperparameters(**kwargs):
  '''
  hyperparameters for plangeneration
  input: result directory
  output: list of hyperparameter objects
  '''

  hyperparam_obj = Hyperparameters(**kwargs)
  return hyperparam_obj


# helper function for finding similar sentence in a corpus given a query
def find_most_similar(query_str, corpus_embedding, translation_lm, num=1):
  query_embedding = translation_lm.encode(query_str, convert_to_tensor=True, device=device)
  # calculate cosine similarity against each candidate sentence in the corpus
  cos_scores = st_utils.pytorch_cos_sim(query_embedding, corpus_embedding)[0].detach().cpu().numpy()

  most_similar_idx = np.argpartition(cos_scores, -1 * num)[-1 * num:]
  most_similar_idx = most_similar_idx[np.argsort(cos_scores[most_similar_idx])][::-1]
  # retrieve high-ranked index and similarity score
  matching_score = cos_scores[most_similar_idx]
  return most_similar_idx, matching_score


def get_similar_eg(query_task, hyperparams, example_tasks, example_task_embeddings, translation_lm):
  '''
  get examples similar to the given task
  input: current task, hyperparameters
  output: list of similar examples
  '''
  if example_tasks == []:
    return False

  example_ids, example_task_scores = find_most_similar(query_task, example_task_embeddings, translation_lm, min(hyperparams.MAX_EXAMPLES, 1))
  print('example_task_scores', [np.round(sc, 3) for sc in example_task_scores])
  example_idx = example_ids[0]
  return example_idx


def get_prompt(inst_AP_variables, example_idx, mode):
  '''
  return a prompt for the planning LLM. Form a prompt for different modes
  input: actino plan variables, the index of the example, mode for prompt
  output: prompt object
  '''
  prompt = {}
  if mode == "trigger explain":
    prompt["system"] = 'You are a helpful assistant. Users will give you input which may or may not have a trigger condition.\n' + \
                       'If present in the user input, a trigger will specify when the command in the input needs to be done. ' + \
                       'Your job is to identify if the input has a trigger and the type of trigger if it does.\n' + \
                       'The triggers can be found in the trigger_list.\n' + \
                       'trigger_list = ["no trigger present", "zone trigger inside <surface>", "timed trigger <time>"]\n' + \
                       'The <surface> token should be replaced by a surface in surface_list that the zone trigger refers to. You see the surfaces from surface_list in the same room as you.\n' + \
                       'The <time> token should be replaced by an unit of time that the timed trigger refers to.'

    prompt["user"] = f'This is the input from the user: {inst_AP_variables.query_task}.\n' + \
                     f'object_list = {inst_AP_variables.query_obj_names}\n' + \
                     f'surface_list = {inst_AP_variables.query_surface_names}\n\n' + \
                     'Does the user input have a trigger? If it does, what is the trigger from trigger_list that satisfies the condition in the user input?'

  if mode == "trigger":
    prompt["system"] = 'You are a helpful assistant. Your job is to map the input to a trigger in the trigger_list.\n' + \
                       'trigger_list = ["no trigger present", "zone trigger inside <surface>", "timed trigger <time>"]\n' + \
                       'The <surface> token should be replaced by a surface in surface_list that the zone trigger refers to. You see the surfaces from surface_list in the same room as you.\n' + \
                       'The <time> token should be replaced by an unit of time that the timed trigger refers to.'

    prompt["user"] = f'This is the input: {inst_AP_variables.query_task}\n' + \
                     f'object_list = {inst_AP_variables.query_obj_names}\n' + \
                     f'surface_list = {inst_AP_variables.query_surface_names}\n\n' + \
                     f'Here is an explanation to guide your output: {inst_AP_variables.explanation}\n' + \
                     'Map the input to a trigger from the trigger_list. Respond using only a trigger in the trigger_list.'

  if mode == "extract command":
    prompt["system"] = 'You are a helpful assistant. You will be given an input with a command to be executed and a condition that needs to be satisfied. ' + \
                       'The condition in the input has been satisfied. Your job is to respond with the command from the input that needs to be executed now.'

    prompt["user"] = f'This is the input: {inst_AP_variables.query_task}\n' + \
                     'The condition in the input has been satisfied. What is the command that needs to be executed now? Respond using a short sentence.'

  if mode == "explain":
    prompt["system"] = f'You are a helpful assistant robot with an arm, capable of performing the actions in action_list.\n' + \
                       f'action_list = [{", ".join(inst_AP_variables.action_list)}]\n' + \
                       f'The <object> token should be replaced by an object in object_list that the action refers to. You see the objects from object_list in the same room as you.\n' + \
                       f'The <surface> token should be replaced by a surface in surface_list that the action refers to. You see the surfaces from surface_list in the same room as you.'

    prompt["user"] = f'This is your task, object_list and surface_list:\n\n' + \
                     f'Task: {inst_AP_variables.query_task}\n' + \
                     f'object_list = {inst_AP_variables.query_obj_names}\n' + \
                     f'surface_list = {inst_AP_variables.query_surface_names}\n'
    
    if inst_AP_variables.feedback is not '':
      prompt["user"] += f'You also received this feedback: {inst_AP_variables.feedback}\n' + \
        'Write a paragraph explaining how you can complete the given task using the action_list, object_list, surface_list and feedback.\n'
    else:
      prompt["user"] += f'Write a paragraph explaining how you can complete the given task using the action_list, object_list and surface_list.\n'

  if mode == "action":
    # [pick <object>, place on <surface>, move to <object>, move to <surface>, wipe <surface>, pour in <object>, done]
    prompt["system"] = f'You are a helpful assistant robot with an arm, capable of performing the actions in action_list.\n' + \
                       f'action_list = [{", ".join(inst_AP_variables.action_list)}]\n' + \
                       f'The <object> token should be replaced by an object in object_list that the action refers to. You see the objects from object_list in the same room as you.\n' + \
                       f'The <surface> token should be replaced by a surface in surface_list that the action refers to. You see the surfaces from surface_list in the same room as you.'

    prompt["user"] = ""
    if example_idx is not False:
      example_task = example_tasks[example_idx]
      example_object_names = example_object_names_list[example_idx]
      example_surfaces = example_surfaces_list[example_idx]
      example_action_plan = example_aps[example_idx]
      prompt["user"] += f'This is an example: \n\n' + \
                        f'Task: {example_task}\n' + \
                        f'object_list = {example_object_names}\n' + \
                        f'surface_list = {example_surfaces}\n\n' + \
                        f'Action plan you used to complete the example: \n'
      for i in range(len(example_action_plan)):
        prompt["user"] += f'{example_action_plan[i]}\n'

    prompt["user"] += f'\nThis is your current task, object_list and surface_list:\n\n' + \
                      f'Task: {inst_AP_variables.query_task}\n' + \
                      f'object_list = {inst_AP_variables.query_obj_names}\n' + \
                      f'surface_list = {inst_AP_variables.query_surface_names}\n\n'

    if inst_AP_variables.explanation is not '':
      prompt["user"] += f'Here is an explanation on how you can solve the task: {inst_AP_variables.explanation}\n\n'

    if inst_AP_variables.feedback is not '':
      prompt["user"] += f'You also received this feedback: {inst_AP_variables.feedback}\n\n'

    prompt["user"] += 'What is the action plan for your current task? ' + \
                      'Respond using only the actions in the action_list and output done after completion.'

    prompt["user"] += '\nAction plan: \n'
    for i in range(len(inst_AP_variables.action_plan)):
      prompt["user"] += f'{inst_AP_variables.action_plan[i]}\n'
    # prompt["user"] += f'Step {len(inst_AP_variables.action_plan)+1}: '

  print("Prompt: ")
  print("System: ", prompt["system"])
  print()
  print("User: ", prompt["user"])
  return prompt


def reduce_samples(samples, log_probs):
  '''
  remove redundant samples for next action
  '''
  samples_dict = {}
  for sample, log_prob in zip(samples, log_probs):
    if sample not in samples_dict:
      samples_dict[sample] = log_prob
    elif log_prob > samples_dict[sample]:
      samples_dict[sample] = log_prob

  samples = []
  log_probs = []
  for sample in samples_dict:
    samples.append(sample)
    log_probs.append(samples_dict[sample])

  return samples, log_probs


def process_action_plans(inst_AP_variables, samples, log_probs, translation_lm, SAMPLE_MATCH_NUM, LLM_ACT):
  '''
  Process sample actions and return possible next actions for this step
  '''
  NL_samples = [] # Possible actions for this step
  action_matching_scores = []
  action_LLM_scores = []

  for plan_sample, log_prob in zip(samples, log_probs):
    plan_sample = plan_sample.split('\n')
    processed_plan = []
    matching_score = 0
    # print("plan_sample:", plan_sample)
    for sample in plan_sample:
      # print("sample: ", sample)
      most_similar_ids, matching_scores = find_most_similar(sample, inst_AP_variables.action_list_embds, translation_lm, SAMPLE_MATCH_NUM)
      most_similar_idx = most_similar_ids[0]
      matching_score += matching_scores[0]
      tx_action = inst_AP_variables.all_actions[most_similar_idx]
      processed_plan.append(tx_action)

    matching_score = matching_score / len(plan_sample)
    # print("processed_plan:", processed_plan)
    # print("matching_score: ", matching_score)
    # print("log_prob: ", log_prob)
    # print()

    NL_samples.append(processed_plan)
    action_matching_scores.append(matching_score)
    action_LLM_scores.append(log_prob)
  return NL_samples, action_matching_scores, action_LLM_scores


# helper function for finding the object in a query
def findObjs(NL_sample):
  NL_action0 = ['Done']
  NL_action1 = ["Pick ", "Place on "]
  NL_action2 = []

  if NL_sample in NL_action0:
    return []

  if any([NL_sample.startswith(NL_action) for NL_action in NL_action1]):
    NL_action = [NL_action for NL_action in NL_action1 if NL_sample.startswith(NL_action)]
    if len(NL_action) > 1:
      raise Exception('multiple actions 1')
      return None
    else:
      sample_objects_NL = NL_sample.replace(NL_action[0], '')
      return [sample_objects_NL]

  if any([(NL_sample.startswith(NL_action[0]) and NL_action[1] in NL_sample) for NL_action in NL_action2]):
    NL_action = [NL_action for NL_action in NL_action2 if (NL_sample.startswith(NL_action[0]) and NL_action[1] in NL_sample)]
    if len(NL_action) > 1:
      raise Exception('multiple actions 2')
      return None
    else:
      sample_objects_NL = NL_sample.split(NL_action[0][0])[-1]
      sample_objects_NL = sample_objects_NL.split(NL_action[0][1])
      return sample_objects_NL

  print("NL_sample for no object match", NL_sample)
  raise Exception('no object match')


def distance_score(last_obj, curr_obj):
  # print("%%%%%%%%", last_obj, curr_obj)
  last_obj_loc = np.array(last_obj[1])
  curr_obj_loc = np.array(curr_obj[1])

  dist = np.linalg.norm(last_obj_loc - curr_obj_loc)
  return np.exp(-dist/10)


def disambiguate_obj(possible_objects_robot, previous_objects, NL_sample, action_plan):
  # score for object state
  possible_obj_scores = []
  for curr_obj in possible_objects_robot:
    curr_obj_scores_temp = []
    for last_obj in previous_objects:
      obj_dist_score = distance_score(last_obj, curr_obj) # score for distance

      curr_obj_scores_temp.append(obj_dist_score)

    if len(curr_obj_scores_temp) == 0:
      curr_obj_scores_temp.append(1.0)

    obj_repeat_score = 0.0 # score to discourage repeating after releasing object
    for step in action_plan:
      if NL_sample == step[0] and curr_obj in step[1]:
        obj_repeat_score -= 1.0
      # print("$$$$$$$$", NL_sample, curr_obj, step, obj_repeat_score)

    possible_obj_scores.append(np.mean(curr_obj_scores_temp) + obj_repeat_score)

  possible_obj_max_score = max(possible_obj_scores)
  return possible_obj_max_score, possible_objects_robot[possible_obj_scores.index(possible_obj_max_score)]


def process_objects(NL_samples, inst_AP_variables, DIST_OBJ):
  object_disamb_scores = []
  objects_robot = [] # Object instances for each action at this step

  for NL_sample in NL_samples: # For each action
    sample_object_disamb_scores = []
    sample_objects_robot = []

    sample_object_names = findObjs(NL_sample)
    # print("########", NL_sample, sample_object_names)
    for object_name in sample_object_names: # For each object in the action
      # Object Disamguation Score
      possible_objects_robot = [obj for obj in inst_AP_variables.query_env if obj[0] == object_name]
      object_disamb_score, object_robot = disambiguate_obj(possible_objects_robot, inst_AP_variables.previous_objects, NL_sample, inst_AP_variables.action_plan)
      # print(f'######## {len(possible_objects_robot)} Object Disamguation Score', object_disamb_score, object_robot)

      sample_object_disamb_scores.append(object_disamb_score)
      sample_objects_robot.append(object_robot)

    if len(sample_objects_robot) == 0:
      sample_object_disamb_scores.append(1.0)

    object_disamb_scores.append(np.mean(sample_object_disamb_scores))
    objects_robot.append(sample_objects_robot)

  return objects_robot, object_disamb_scores


def check_action_plan(inst_AP_variables):
  # ["pick <object>", "place on <surface>", "pour in <object>", "wave at me", "done", "action plan"]
  # rule 1: place before another pick
  # rule 2: pick before place
  # rule 3: pick before pour
  # rule 4: place before wave

  picked_object = inst_AP_variables.attached_object_name
  for step, action in enumerate(inst_AP_variables.action_plan):
    if action.startswith("pick "):
      current_object = action[5:]
      if picked_object != "":
        inst_AP_variables.feedback += 'If you want to pick, you should place the picked object on a surface before picking another object. '
        # inst_AP_variables.action_plan = inst_AP_variables.action_plan[:step]
        inst_AP_variables.action_plan = []
      picked_object = current_object

    if action.startswith("place on "):
      # current_surface = action[9:]
      if picked_object == "":
        inst_AP_variables.feedback += 'You should pick up an object before placing it on a surface'
        # inst_AP_variables.action_plan = inst_AP_variables.action_plan[:step]
        inst_AP_variables.action_plan = []
      picked_object = ""

    if action.startswith("pour in "):
      # current_object = action[8:]
      if picked_object == "":
        inst_AP_variables.feedback += 'If you want to pour in, you should pick up an object before pouring anything from it in another object'
        # inst_AP_variables.action_plan = inst_AP_variables.action_plan[:step]
        inst_AP_variables.action_plan = []

    if action == "wave at me":
      if picked_object != "":
        inst_AP_variables.feedback += 'If you want to wave, you should place the picked object on a surface before waving'
        # inst_AP_variables.action_plan = inst_AP_variables.action_plan[:step]
        inst_AP_variables.action_plan = []

  if inst_AP_variables.feedback == "":
    return True, inst_AP_variables
  return False, inst_AP_variables


def generate_action_plan(inst_AP_variables, generator, translation_lm, hyperparams):
  # try:
  samples, log_probs = generator(inst_AP_variables.prompt, hyperparams.sampling_params, hyperparams.max_tokens, hyperparams.stop) # query Planning LM for single-step action candidates
  samples, log_probs = reduce_samples(samples, log_probs)
  NL_samples, action_matching_scores, action_LLM_scores = process_action_plans(inst_AP_variables, samples, log_probs, translation_lm, hyperparams.SAMPLE_MATCH_NUM, hyperparams.LLM_ACT)

  scores = [hyperparams.WT_ACT * action_matching + hyperparams.LLM_ACT * action_LLM for action_matching, action_LLM in zip(action_matching_scores, action_LLM_scores)]
  print("-------------------------", samples, "  ", NL_samples, "  ", action_matching_scores, "  ", action_LLM_scores, "  ", scores)
  max_score_idx = np.argmax(scores)

  # if "done" in NL_samples:
  #   inst_AP_variables.action_plan.append("done")
  #   return True

  inst_AP_variables.action_plan = NL_samples[max_score_idx]

  return True
  # except:
  #   return False