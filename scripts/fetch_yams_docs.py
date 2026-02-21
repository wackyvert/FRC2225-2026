import os
import shutil
import subprocess
import sys
import tempfile

def run_command(command, cwd=None):
    """Run a shell command."""
    try:
        # We use shell=True, but we don't rely on CWD for relative paths in the command
        subprocess.check_call(command, shell=True, cwd=cwd)
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {command}")
        sys.exit(1)

def main():
    repo_url = "https://github.com/Yet-Another-Software-Suite/YAMS.git"
    
    # Use a system temporary directory to avoid UNC/Network share issues with CMD.EXE
    with tempfile.TemporaryDirectory() as temp_dir:
        print(f"Using temporary directory: {temp_dir}")
        
        # 1. Clone the repository into the absolute temp path
        print(f"Cloning {repo_url}...")
        run_command(f"git clone --depth 1 {repo_url} \"{temp_dir}\"")

        # 2. Locate Destination (Relative to this script)
        # Get the directory of the script file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Go up one level to project root, then into docs/yams
        project_root = os.path.dirname(script_dir)
        target_dir = os.path.join(project_root, "docs", "yams")

        # 3. Clean up existing target
        if os.path.exists(target_dir):
            print(f"Cleaning up existing target directory: {target_dir}")
            shutil.rmtree(target_dir)

        # 4. Locate Documentation in the clone
        source_docs = os.path.join(temp_dir, "docs")
        
        if not os.path.exists(source_docs):
            print(f"Warning: 'docs' folder not found in {source_docs}. Searching for .md files recursively...")
            
            os.makedirs(target_dir, exist_ok=True)
            
            file_count = 0
            for root, dirs, files in os.walk(temp_dir):
                # Skip .git directory
                if ".git" in root:
                    continue
                    
                for file in files:
                    if file.endswith(".md"):
                        src_file = os.path.join(root, file)
                        # Create relative path structure from the temp dir
                        rel_path = os.path.relpath(src_file, temp_dir)
                        dest_file = os.path.join(target_dir, rel_path)
                        
                        os.makedirs(os.path.dirname(dest_file), exist_ok=True)
                        shutil.copy2(src_file, dest_file)
                        file_count += 1
            print(f"Copied {file_count} Markdown files to {target_dir}")
            
        else:
            # Copy the docs folder
            print(f"Found docs folder. Copying to {target_dir}...")
            shutil.copytree(source_docs, target_dir)
            
            # Also copy README.md from root if it exists
            root_readme = os.path.join(temp_dir, "README.md")
            if os.path.exists(root_readme):
                 shutil.copy2(root_readme, os.path.join(target_dir, "README.md"))
            print("Documentation copied successfully.")

    # Temp dir is auto-cleaned by the context manager

if __name__ == "__main__":
    main()