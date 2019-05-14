# Unique Id of the user creating the task
string user_id

# Id of the patient to whom the delivery is to be made
string patient_id

# Information about the category, type and name of the product
# listed in sequential form in the array
# for ex. Hot milo will be described as >
# ["Hot Beverages","Milo"]
string[] product_info

# Unique identifier to keep track of the task between nodes
string uuid

string status
