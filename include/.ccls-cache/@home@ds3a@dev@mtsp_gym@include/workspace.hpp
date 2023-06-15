#ifndef WORKSPACE_H_
#define WORKSPACE_H_

namespace mtsp_gym {
    class Workspace {
    private:
        // in meters
        float height;
        float width;

        /*
         * TODO keep note of obstacles, payload, and stuff?
         */
    public:
        Workspace();
    };
}


#endif // WORKSPACE_H_
