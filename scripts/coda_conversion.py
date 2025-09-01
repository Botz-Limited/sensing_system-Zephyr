
import os
import re

# Dictionary mapping mermaid diagram types to Coda formulas
DIAGRAM_TYPE_TO_FORMULA = {
    "graph": "DrawFlowchart",
    "sequenceDiagram": "DrawSequenceDiagram",
    "stateDiagram-v2": "DrawStateDiagram",
    "gantt": "DrawGanttChart",
    "pie": "DrawPieChart",
    "classDiagram": "DrawClassDiagram",
    "erDiagram": "DrawEntityRelationship",
    "journey": "DrawUserJourney",
    "requirementDiagram": "DrawRequirementDiagram",
    "gitGraph": "DrawGitGraph",
    "c4": "DrawC4Diagram",
    "mindmap": "DrawMindmap",
    "quadrantChart": "DrawQuadrantChart",
    "timeline": "DrawTimeline",
    "sankey-beta": "DrawSankey",
    "xychart-beta": "DrawXYChart",
    "block-beta": "DrawBlockDiagram",
    "packet-beta": "DrawPacketDiagram",
    "kanban-beta": "DrawKanban",
    "architecture-beta": "DrawArchitectureDiagram",
}

def convert_mermaid_to_coda(mermaid_code):
    # Extract the diagram type from the first line
    first_line = mermaid_code.strip().split('\n')[0].strip()
    diagram_type = first_line.split(' ')[0]

    # Get the corresponding Coda formula
    coda_formula = DIAGRAM_TYPE_TO_FORMULA.get(diagram_type)

    if coda_formula:
        # Escape double quotes in the mermaid code
        escaped_code = mermaid_code.replace('"', '\\"')
        # Wrap the mermaid code in the Coda formula
        return f'`{coda_formula}("{escaped_code}")`'
    else:
        return None

def process_markdown_file(input_file_path, output_file_path):
    with open(input_file_path, 'r') as f:
        content = f.read()

    # Find all mermaid code blocks
    mermaid_blocks = re.findall(r'```mermaid(.*?)```', content, re.DOTALL)

    # Convert each mermaid block to a Coda formula
    for block in mermaid_blocks:
        coda_formula = convert_mermaid_to_coda(block)
        if coda_formula:
            # Replace the original mermaid block with the Coda formula
            content = content.replace(f'```mermaid{block}```', coda_formula)

    # Write the converted content to the output file
    with open(output_file_path, 'w') as f:
        f.write(content)

if __name__ == "__main__":
    docs_directory = "docs"
    for filename in os.listdir(docs_directory):
        if filename.endswith(".md"):
            input_path = os.path.join(docs_directory, filename)
            output_path = os.path.join(docs_directory, f"{os.path.splitext(filename)[0]}.coda")
            process_markdown_file(input_path, output_path)
