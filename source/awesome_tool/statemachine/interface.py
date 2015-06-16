

def open_folder_cmd_line(query):
    user_input = raw_input(query + ': ')
    if len(user_input) == 0:
        return None
    return user_input

open_folder_func = open_folder_cmd_line


def create_folder_cmd_line(query):
    import os
    user_input = raw_input(query + ': ')
    if len(user_input) == 0 or os.path.exists(user_input):
        return None

    try:
        os.makedirs(user_input)
    except OSError:
        return None
    return user_input

create_folder_func = create_folder_cmd_line


def show_notice(query):
    raw_input(query)

show_notice_func = show_notice
