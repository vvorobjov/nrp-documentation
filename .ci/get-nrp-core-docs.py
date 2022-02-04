import requests
import argparse
import sys
import json
import re

RepositoryLocation = "https://nexus.neurorobotics.ebrains.eu/"
RepositoryAPI = RepositoryLocation + "service/rest/v1/"
RepositoryName = "nrp-storage"

parser = argparse.ArgumentParser()
parser.add_argument("topic_branch", help="TOPIC_BRANCH from Jenkinsfile: feature branch name")
parser.add_argument("default_branch", help="DEFAULT_BRANCH from Jenkinsfile: the name of the branch to use if TOPIC_BRANCH is unavailable")
parser.add_argument("username", help="Repository username")
parser.add_argument("password", help="Repository password")

## Input arguments parser

args = parser.parse_args()

def download_file(url, local_filename):
    with requests.get(url, stream=True, auth=(args.username, args.password)) as r:
        r.raise_for_status()
        with open(local_filename, 'wb') as f:
            for chunk in r.iter_content(chunk_size=8192): 
                # If you have chunk encoded response uncomment if
                # and set chunk_size parameter to None.
                #if chunk: 
                f.write(chunk)
    return local_filename

def find_nrp_core_docs_asset(assets, branch):
    downloadUrl = None
    for item in assets["items"]:
        # print(item["path"])
        m = re.search("nrp-core/nrp-core-docs/{}/.*.zip$".format(branch), item["path"])
        if not m:
            continue
        if not (item["contentType"] == "application/zip"):
            continue

        downloadUrl = item["downloadUrl"]
    return downloadUrl

## Script
topicDownloadURL = None
defaultDownloadURL = None

url = RepositoryAPI + "search/assets"
continuationToken = "First page token"
payload = {"repository": RepositoryName}
print(url)

while continuationToken:

    response = requests.get(url, params=payload, auth=(args.username, args.password))

    print(response.status_code)
    if response.status_code != 200:
        sys.exit("Couldn't get the list of repository assets: {}".format(url))

    assets = json.loads(response.text)
    # print(assets)

    # Get continuationToken for the next pages
    continuationToken = assets["continuationToken"]    
    # Encode next page url request
    payload = {"repository": RepositoryName, "continuationToken": continuationToken}

    # Try to fetch the topic data on this page
    topicDownloadURL = find_nrp_core_docs_asset(assets, args.topic_branch)
    # Exit if asset is found
    if topicDownloadURL:
        break

    # Try to fetch the default data on this page if not yet found
    if not defaultDownloadURL:
        defaultDownloadURL = find_nrp_core_docs_asset(assets, args.default_branch)

## The pages are over, try to download
if topicDownloadURL:
    download_file(topicDownloadURL, "nrp-core-docs.zip")
elif defaultDownloadURL:
    download_file(defaultDownloadURL, "nrp-core-docs.zip")
else:
    sys.exit("Couldn't find assets {} or {}".format(args.topic_branch, args.default_branch))
