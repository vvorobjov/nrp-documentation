
NRP Backend Proxy debugging
===========================

You can debug the NRP Backend Proxy software if you :ref:`installed the NRP from source <source-installation>`. 

NRP Backend Proxy is a Node.js application and we offer a way to debug it with VS Code editor, that can be modified for your favourite IDE.
In case you followed :ref:`source installation instruction <source-installation>`, then add the following lines into the file :code:`$HBP/nrpBackendProxy/.vscode/launch.json` (create the file if it's absent):

.. code-block:: json

    {
        "configurations": [
            {
                "type": "node",
                "request": "launch",
                "name": "Launch Program",
                "program": "node_modules/ts-node/dist/bin.js",
                "args": ["app.ts"],
                "preLaunchTask": "tsc: build - tsconfig.json",
                "outFiles": ["${workspaceFolder}/**/*.js"]
            }
        ]
    }

And then debug it as usual in VS Code editor. More information on debugging in VS Code can be found `here <https://code.visualstudio.com/docs/editor/debugging>`_.

.. note:: Before starting the NRP, remember to remove :code:`cle-proxy` command from :code:`cle-start`. The aliases are defined in :code:`"$HBP"/user-scripts/nrp_aliases` and sourced in :code:`$HOME/.bashrc`.
