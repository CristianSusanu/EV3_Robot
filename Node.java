import java.util.ArrayList;

public class Node
{
	private String Type = "NA"; // "Empty" || "Wall" || "No-Go" || "Finish" || "Gap"
	private int coordX;
	private int coordY;
	private Boolean Explored = false;
	private ArrayList<Node> unvisitedChildren = new ArrayList<Node>();
	private ArrayList<Node> Children = new ArrayList<Node>();
	private Node Previous;
	private Node Parent = null;
	private String DiscoveryOrientation;
	private ArrayList<Node> Parents = new ArrayList<Node>();
	
	public int StartToNode = 0;
	public int NodeToEnd = 0;
	public int totalCost = 0;
	
	public Node(int coordX, int coordY) {
		this.coordX = coordX;
		this.coordY = coordY;
	}
	
	public String toString() {
		return coordX + "|" + coordY;
	}
	
	public void setType(String Type) {
		 this.Type = Type;
	}
	
	public String getType() {
		return Type;
	}

	public void setDiscovery(String ori) {
		DiscoveryOrientation = ori;
	}
	
	public String getDiscovery() {
		return DiscoveryOrientation;
	}
	
	public void setExplored(Boolean Explored) {
		this.Explored = Explored;
	}
	
	public Boolean getExplored() {
		return Explored;
	}
	
	public int getX() {
		return coordX;
	}
	
	public int getY() {
		return coordY;
	}
	
	public void setPrev(Node Previous) {
		this.Previous = Previous;
	}
	
	public Node getPrev() {
		return Previous;
	}
	
	public void addUnvisChild(Node Child) {
		unvisitedChildren.add(Child);
	}
	
	public ArrayList<Node> getUnvisChildren() {
		return unvisitedChildren;
	}
	
	public void addChild(Node aStarChild) {
		Children.add(aStarChild);
	}
	
	public ArrayList<Node> getChildren() {
		return Children;
	}
	
	public void setParent(Node aStarParent) {
		Parent = aStarParent;
	}
	
	public Node getParent() {
		return Parent;
	}

	public void addParent(Node Parent) {
		Parents.add(Parent);
	}
	
	public ArrayList<Node> getParents() {
		return Parents;
	}
	
	public void resetNodeCost() {
		totalCost = 0;
		StartToNode = 0;
		NodeToEnd = 0;
	}
	
}
