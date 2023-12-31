def cloneRepoTopic(folder, repoUrl, topicBranch, defaultBranch) {
// cloneRepoTopic: 
//      1 - directory to checkout
//      2 - repo
//      3 - name of topic branch
//      4 - default branch if topic unavailable
//      5 - username for chown
    dir(folder) {
        try {
            echo "${folder}: Trying to checkout branch ${topicBranch}."
            checkout([
                $class: "GitSCM",
                branches: [[name: topicBranch]], 
                extensions: [], 
                userRemoteConfigs: [[
                    credentialsId: "${GIT_SSH_KEY}", 
                    url: repoUrl
                ]]
            ])
        }
        catch (e) {
            echo "${folder}: Branch ${topicBranch} is not available, getting ${defaultBranch} instead."
            checkout([
                $class: "GitSCM",
                branches: [[name: defaultBranch]], 
                extensions: [], 
                userRemoteConfigs: [[
                    credentialsId: "${GIT_SSH_KEY}", 
                    url: repoUrl
                ]]
            ])
        }
    }
}
def selectTopicBranch(branch_name, change_branch){
    if (change_branch){
        return change_branch
    }
    else {
        return branch_name
    }
}

def docs_version
pipeline
{
    environment
    {
        USER_SCRIPTS_DIR = "nrp-user-scripts"
        NRP_BACKEND_DIR = "nrp-backend"
        DOCS_DIR = "nrp-documentation"
        GIT_CHECKOUT_DIR = "${env.DOCS_DIR}"

        NexusRegistry = "https://${env.NEXUS_REGISTRY_IP}/"

        // If parameter BRANCH_NAME is set, use it as topic,
        // otherwise, use env.BRANCH_NAME or env.CHANGE_BRANCH (if it's PR)
        TOPIC_BRANCH = selectTopicBranch(selectTopicBranch(env.BRANCH_NAME, env.CHANGE_BRANCH), params.BRANCH_NAME)

        // If parameter BASE_BRANCH_NAME is set, use it as default branch
        // otherwise, use development
        DEFAULT_BRANCH = selectTopicBranch('development', params.BASE_BRANCH_NAME)
    }
    agent {
        docker {
            label 'cd_label'
            // NEXUS_REGISTRY_IP and NEXUS_REGISTRY_PORT are Jenkins global variables
            image "${env.NEXUS_REGISTRY_IP}:${env.NEXUS_REGISTRY_PORT}/nrp:development"
            registryUrl "http://${env.NEXUS_REGISTRY_IP}:${env.NEXUS_REGISTRY_PORT}"
            registryCredentialsId 'nexusadmin'
            args '--entrypoint="" -u root --privileged'
            alwaysPull true
        }
    }
    options { 
        // Skip code checkout prior running pipeline (only Jenkinsfile is checked out)
        skipDefaultCheckout true

        ansiColor('xterm')
    }

    stages
    {
        stage('Cloning code')
        {
            when
            {
                expression { true }
            }
            steps{
                sh "rm -rf *"

                sh "echo TOPIC_BRANCH: ${env.TOPIC_BRANCH}"
                sh "echo DEFAULT_BRANCH: ${env.DEFAULT_BRANCH}"

                // Checkout main project to GIT_CHECKOUT_DIR
                dir(env.GIT_CHECKOUT_DIR) {
                    checkout scm
                }

                cloneRepoTopic(env.USER_SCRIPTS_DIR, 'git@bitbucket.org:hbpneurorobotics/nrp-user-scripts.git', env.TOPIC_BRANCH, env.DEFAULT_BRANCH)
                cloneRepoTopic(env.NRP_BACKEND_DIR, 'git@bitbucket.org:hbpneurorobotics/nrp-backend.git', env.TOPIC_BRANCH, env.DEFAULT_BRANCH)

                sh "git config --global --add safe.directory '*'"
            }
        }
        stage('Gathering Docs')
        {
            when
            {
                expression {  true }
            }
            steps
            {
                dir(env.DOCS_DIR)
                {
                    withCredentials([ \
                        usernamePassword(credentialsId: 'nexusadmin', usernameVariable: 'USER', passwordVariable: 'PASSWORD')
                    ])
                    {
                        sh 'python3 ./.ci/get-nrp-core-docs.py $TOPIC_BRANCH $DEFAULT_BRANCH $NexusRegistry $USER $PASSWORD'
                    }
                    
                    sh "rm -rf /home/bbpnrsoa/.opt/platform_venv"
                    sh "export BUILD_NUMBER; export HBP=$WORKSPACE; bash ./.ci/build.bash ${params.RELEASE}"
                    archiveArtifacts artifacts: "_build/html/**/*"
                    recordIssues enabledForFailure: true, tools: [sphinxBuild(pattern: 'sphinx_w.txt')], qualityGates: [[threshold: 8, type: 'TOTAL', unstable: true]]
                }
            }
        }
        stage('Deploy with ansible')
        {
            when
            {
                expression { return params.DEPLOY }
            }
            steps
            {
                script
                {
                    dir(env.DOCS_DIR)
                    {
                        docs_version = readFile "version"
                        docs_version = docs_version.trim()
                        
                        sh '''
                        sudo apt install ca-certificates
                        sudo apt-add-repository --yes --update ppa:ansible/ansible
                        sudo apt update 
                        sudo apt install -y software-properties-common ansible-core
                        ansible-galaxy collection install community.general
                        ansible-galaxy collection install ansible.posix
                        '''
                        ansiblePlaybook(credentialsId: "website_ansible_key", \
                                        colorized: true, \
                                        inventory: 'ansible/hosts', \
                                        playbook: 'ansible/deploy_docs.yml',  \
                                        become : true ,  \
                                        extraVars: [ansible_become_pass :  "${params.DEPLOY_PASS}" , \
                                                    docs_version : "${docs_version}", \
                                                    sphinx_build_dir :  '${WORKSPACE}/${DOCS_DIR}/_build/', \
                                                    link_latest : "${params.LATEST}", \
                                                    var_release : "${params.RELEASE}" ] )
                        currentBuild.description = readFile "ansible/destination.txt"
                        currentBuild.description = "${env.TOPIC_BRANCH}: " + currentBuild.description
                    }
                }
            }
        }
    }

}