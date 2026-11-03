def is_selected_topic(topic,allowed):
    return topic in allowed

def health_level(age_s,warn_s,error_s):
    return 2 if age_s >= error_s else 1 if age_s >= warn_s else 0
