Import("env")
import os

project_dir = env.subst("$PROJECT_DIR")

home  = open(os.path.join(project_dir,'data','home.html'),  encoding='utf-8').read()
about = open(os.path.join(project_dir,'data','about.html'), encoding='utf-8').read()
net   = open(os.path.join(project_dir,'data','networkconfig.html'), encoding='utf-8').read()

do = 'R"V0G0N('
dc = ')V0G0N"'

out = '#pragma once\n\n'
out += '// Inline HTML pages auto-generated from data/ folder by tools/gen_inline_pages.py\n\n'
out += 'const char* AxRobotControlWebServerPage = ' + do + '\n' + home + '\n' + dc + ';\n\n'
out += 'const char* AboutPage2 = ' + do + '\n' + about + '\n' + dc + ';\n\n'
out += 'const char* NetworkConfigPage = ' + do + '\n' + net + '\n' + dc + ';\n'

out_path = os.path.join(project_dir,'src','InlinePages.h')
with open(out_path,'w',encoding='utf-8') as f:
    f.write(out)
print("gen_inline_pages: wrote", out_path)
