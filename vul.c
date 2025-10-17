int main(int argc, char **argv)
{
    {
        // ruleid: dynamic-library-path
        dlopen(argv[1], RTLD_NOW);
    }
 
    {
        // ok: dynamic-library-path
        void *handle = dlopen("foo.so", RTLD_NOW);
        // ruleid: dynamic-library-path
        dlsym(handle, argv[2]);
    }
 
    {
        char buffer[128];
        char *filename = gets(buffer);
        // ruleid: dynamic-library-path
        dlmopen(NULL, filename, RTLD_NOW);
    }
 
    {
        char *filename = shared_library_path();
        // ok: dynamic-library-path
        dlopen(filename, RTLD_NOW);
    }
 
    {
        std::string path = "/usr/local/";
        path.append(argv[1]);
        path.append(".so");
        // ruleid: dynamic-library-path
        dlopen(path.c_str(), RTLD_NOW);
    }
}